#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
import numpy as np

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid

class OccupancyGrid2d:
    def __init__(self):
        self._initialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Initialize map and periodic visualization timer.
        self._map = np.zeros((self._x_num, self._y_num))
        # This timer calls VisualizeCallback every second, so the map updates even if no scans arrive.
        self._visualization_timer = rospy.Timer(rospy.Duration(1.0), self.VisualizeCallback)

        self._initialized = True
        rospy.loginfo("%s: Initialized successfully.", self._name)
        return True

    def LoadParameters(self):
        # Downsample fraction
        self._random_downsample = rospy.get_param("~random_downsample", 0.1)

        # Dimensions and resolution
        self._x_num = rospy.get_param("~x/num", 100)
        self._x_min = rospy.get_param("~x/min", -10.0)
        self._x_max = rospy.get_param("~x/max", 10.0)
        self._x_res = (self._x_max - self._x_min) / self._x_num

        self._y_num = rospy.get_param("~y/num", 100)
        self._y_min = rospy.get_param("~y/min", -10.0)
        self._y_max = rospy.get_param("~y/max", 10.0)
        self._y_res = (self._y_max - self._y_min) / self._y_num

        # Update parameters
        self._occupied_update = self.ProbabilityToLogOdds(rospy.get_param("~update/occupied", 0.7))
        self._occupied_threshold = self.ProbabilityToLogOdds(rospy.get_param("~update/occupied_threshold", 0.97))
        self._free_update = self.ProbabilityToLogOdds(rospy.get_param("~update/free", 0.3))
        self._free_threshold = self.ProbabilityToLogOdds(rospy.get_param("~update/free_threshold", 0.03))

        # Topics and frames
        self._sensor_topic = rospy.get_param("~topics/sensor", "/scan")
        self._vis_topic = rospy.get_param("~topics/vis", "/map_visualization")
        self._sensor_frame = rospy.get_param("~frames/sensor", "base_footprint")
        self._fixed_frame = rospy.get_param("~frames/fixed", "odom")    

        rospy.loginfo("%s: Parameters loaded: sensor_topic=%s, vis_topic=%s, sensor_frame=%s, fixed_frame=%s",
                      self._name, self._sensor_topic, self._vis_topic, self._sensor_frame, self._fixed_frame)
        return True

    def RegisterCallbacks(self):
        # Subscriber for sensor data
        self._sensor_sub = rospy.Subscriber(
            self._sensor_topic,
            LaserScan,
            self.SensorCallback,
            queue_size=1)

        # Publisher for visualization (marker)
        self._vis_pub = rospy.Publisher(
            self._vis_topic,
            Marker,
            queue_size=10)

        # Publisher for occupancy grid
        self._map_pub = rospy.Publisher(
            "/map",
            OccupancyGrid,
            queue_size=1)
        
        return True

    def SensorCallback(self, msg):
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])

        if abs(pose.transform.translation.z) > 0.05:
            rospy.logwarn("%s: Turtlebot is not on ground plane.", self._name)
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)

        # Update the map based on the scan data.
        for idx, r in enumerate(msg.ranges):
            if np.random.rand() > self._random_downsample or np.isnan(r):
                continue
            ith_angle = msg.angle_min + msg.angle_increment * idx
            fixed_angle = ith_angle + yaw

            if r < msg.range_min or r > msg.range_max:
                continue

            step = min(self._x_res, self._y_res)
            last_voxel = True  # The first voxel from the hit point is occupied
            for dist in np.arange(r, 0, -step):
                x = sensor_x + dist * np.cos(fixed_angle)
                y = sensor_y + dist * np.sin(fixed_angle)
                grid_x, grid_y = self.PointToVoxel(x, y)

                if 0 <= grid_x < self._x_num and 0 <= grid_y < self._y_num:
                    if last_voxel:
                        # Occupied cell
                        self._map[grid_x, grid_y] = min(
                            self._occupied_threshold, self._map[grid_x, grid_y] + self._occupied_update)
                        last_voxel = False
                    else:
                        # Free cell
                        self._map[grid_x, grid_y] = max(
                            self._free_threshold, self._map[grid_x, grid_y] + self._free_update)

        # After updating the map, visualize and publish occupancy grid.
        self.Visualize()
        self.PublishOccupancyGrid()

    def VisualizeCallback(self, event):
        self.Visualize()

    def Visualize(self):
        # Create a Marker for visualization.
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self._x_res
        m.scale.y = self._y_res
        m.scale.z = 0.01

        # Set a valid orientation to avoid warnings
        m.pose.orientation.w = 1.0

        for ii in range(self._x_num):
            for jj in range(self._y_num):
                p = Point()
                p.x, p.y = self.VoxelCenter(ii, jj)
                p.z = 0.0
                m.points.append(p)
                m.colors.append(self.Colormap(ii, jj))

        self._vis_pub.publish(m)

    def PublishOccupancyGrid(self):
        # Convert the internal log-odds map to an OccupancyGrid using the same logic as the marker
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = self._fixed_frame
        grid_msg.info.resolution = self._x_res
        grid_msg.info.width = self._x_num
        grid_msg.info.height = self._y_num
        grid_msg.info.origin.position.x = self._x_min
        grid_msg.info.origin.position.y = self._y_min
        grid_msg.info.origin.orientation.w = 1.0

        flat_map = self._map.flatten()
        occupancy_data = []
        for val in flat_map:
            # Compute probability just like in the Colormap function
            p = self.LogOddsToProbability(val)
            # Map probability p (0.0 to 1.0) directly to occupancy (0 to 100)
            occ = int(round(p * 100))
            occupancy_data.append(occ)

        grid_msg.data = occupancy_data
        self._map_pub.publish(grid_msg)

    def PointToVoxel(self, x, y):
        grid_x = int((x - self._x_min) / self._x_res)
        grid_y = int((y - self._y_min) / self._y_res)
        return (grid_x, grid_y)

    def VoxelCenter(self, ii, jj):
        center_x = self._x_min + (0.5 + ii) * self._x_res
        center_y = self._y_min + (0.5 + jj) * self._y_res
        return (center_x, center_y)

    def ProbabilityToLogOdds(self, p):
        return np.log(p / (1.0 - p))

    def LogOddsToProbability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    def Colormap(self, ii, jj):
        p = self.LogOddsToProbability(self._map[ii, jj])
        c = ColorRGBA()
        c.r = p
        c.g = 0.1
        c.b = 1.0 - p
        c.a = 0.75
        return c


if __name__ == "__main__":
    rospy.init_node("occupancy_grid_2d")
    node = OccupancyGrid2d()
    if node.Initialize():
        rospy.spin()

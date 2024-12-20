#!/usr/bin/env python3
import rospy
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener, LookupException
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
import heapq

class ARNavigationToMarker:
    def __init__(self):
        rospy.init_node('ar_marker_navigator', anonymous=True)

        # Subscribers
        rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.occupancy_grid_callback)
        rospy.Subscriber('/ar_pose_marker', PoseStamped, self.ar_marker_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Parameters
        self.grid = None
        self.width = 0
        self.height = 0
        self.resolution = 0
        self.origin = (0, 0)
        self.marker_position = None
        self.current_position = None

        rospy.loginfo("AR Marker Navigator Initialized")

    def occupancy_grid_callback(self, msg):
        """Callback to process occupancy grid data."""
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def ar_marker_callback(self, msg):
        """Callback to process AR marker position."""
        try:
            transform = self.tf_buffer.lookup_transform("map", msg.header.frame_id, rospy.Time(0))
            self.marker_position = self.transform_pose(msg.pose, transform)
            rospy.loginfo(f"AR Marker Position: {self.marker_position}")
        except LookupException as e:
            rospy.logwarn(f"Transform lookup failed: {e}")

    def transform_pose(self, pose, transform):
        """Transform pose to the map frame."""
        x = transform.transform.translation.x + pose.position.x
        y = transform.transform.translation.y + pose.position.y
        return (x, y)

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        grid_x = int((x - self.origin[0]) / self.resolution)
        grid_y = int((y - self.origin[1]) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates."""
        x = grid_x * self.resolution + self.origin[0]
        y = grid_y * self.resolution + self.origin[1]
        return x, y

    def a_star(self, start, goal):
        """A* algorithm for path planning."""
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                # Reconstruct path
                path = []
                while current:
                    path.append(current)
                    current = came_from.get(current, None)
                return path[::-1]

            for neighbor in self.get_neighbors(*current):
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current

        return None  # No path found

    def heuristic(self, a, b):
        """Heuristic function (Euclidean distance)."""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, x, y):
        """Get neighboring cells that are valid for movement."""
        neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        valid_neighbors = [
            (nx, ny) for nx, ny in neighbors
            if 0 <= nx < self.width and 0 <= ny < self.height and self.grid[ny, nx] == 0
        ]
        return valid_neighbors

    def navigate_to_marker(self):
        """Navigate to the detected AR marker."""
        if self.grid is None or self.marker_position is None:
            rospy.logwarn("Occupancy grid or marker position not available.")
            return

        # Convert start and goal to grid indices
        start = self.world_to_grid(*self.current_position)
        goal = self.world_to_grid(*self.marker_position)

        rospy.loginfo(f"Start: {start}, Goal: {goal}")

        # Plan path using A*
        path = self.a_star(start, goal)
        if path is None:
            rospy.logerr("No path found to the goal!")
            return

        rospy.loginfo(f"Path: {path}")

        # Follow the path
        for cell in path:
            world_pos = self.grid_to_world(*cell)
            self.move_to_position(world_pos)

    def move_to_position(self, target):
        """Move the robot to the target position."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_position is None:
                continue

            dx = target[0] - self.current_position[0]
            dy = target[1] - self.current_position[1]
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.1:  # Target reached
                self.stop_robot()
                rospy.loginfo(f"Reached position: {target}")
                break

            # Move towards the target
            vel_msg = Twist()
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = math.atan2(dy, dx)
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()

    def stop_robot(self):
        """Stop the robot."""
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        navigator = ARNavigationToMarker()
        rospy.sleep(2)  # Wait for data
        navigator.navigate_to_marker()
    except rospy.ROSInterruptException:
        pass

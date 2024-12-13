#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import LookupException
from tf2_ros import ExtrapolationException
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan



class ARNavigationDiscrete:
    def __init__(self):
        rospy.init_node('ar_navigation_discrete', anonymous=True)

        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        # Publisher for robot velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to odometry for robot's current pose
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Current pose from odometry
        self.current_pose = None

        # Thresholds for movement
        self.distance_threshold = 0.1  # Meters
        self.slow_down_distance = 0.5  # Distance within which to start slowing down
        self.min_speed = 0.01          # Minimum speed near the turn
        self.max_speed = 0.05           # Maximum speed for straight-line motion
        self.angle_threshold = 0.05    # Radians

        rospy.loginfo("AR Navigation with discrete straight-line travel initialized.")

    def odom_callback(self, msg):
        """Update the robot's current pose from odometry."""
        self.current_pose = msg.pose.pose

    def compute_speed(self, distance):
        """Compute speed based on remaining distance."""
        if distance < self.slow_down_distance:
            # Scale speed linearly as the robot approaches the target
            return max(self.min_speed, (distance / self.slow_down_distance) * self.max_speed)
        return self.max_speed

    def move_straight(self, target_x, target_y):
        """Move straight to a target position."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue

            # Compute the distance to the target
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance <= self.distance_threshold:
                rospy.loginfo("Reached target position.")
                break

            # Compute dynamic speed
            speed = self.compute_speed(distance)
            rospy.loginfo(f"Distance: {distance:.2f}, Speed: {speed:.2f}")

            # Publish velocity to move straight
            vel_msg = Twist()
            vel_msg.linear.x = speed  # Adjusted speed
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()

        # Stop the robot after reaching the target
        self.stop_robot()

    def turn_to_angle(self, target_angle):
        """Turn to a specific orientation (yaw)."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue

            # Extract current yaw from orientation
            orientation_q = self.current_pose.orientation
            _, _, current_yaw = euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )

            # Compute the difference between current and target yaw
            angle_diff = self.normalize_angle(target_angle - current_yaw)

            if abs(angle_diff) <= self.angle_threshold:
                rospy.loginfo("Reached target orientation.")
                break

            # Publish velocity to rotate
            vel_msg = Twist()
            vel_msg.angular.z = 0.5 if angle_diff > 0 else -0.5  # Turn direction
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()

        # Stop the robot after turning
        self.stop_robot()

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def get_clear_direction(self, laser_data, granularity=1, last_direction=None):
        """
        Find the direction of the largest clear space using laser scan data.

        Parameters:
        - laser_data: LaserScan message.
        - granularity: Angle step size in degrees. Smaller values increase precision.
        - last_direction: The last chosen direction (optional).

        Returns:
        - direction: The angle (in radians) of the largest clear space.
        """
        ranges = np.array(laser_data.ranges)
        max_range = laser_data.range_max
        angles = np.linspace(laser_data.angle_min, laser_data.angle_max, len(ranges))

        # Mask invalid ranges
        valid_ranges = np.where((ranges > 0.1) & (ranges < max_range), ranges, 0)

        # Calculate step size
        total_angles = 360  # Assuming a full 360° scan
        step_size = max(1, int(len(angles) / (total_angles / granularity)))

        # Resample angles and ranges to match the desired granularity
        angles_resampled = angles[::step_size]
        ranges_resampled = valid_ranges[::step_size]

        # Avoid previously selected direction if stuck
        if last_direction is not None:
            for i, angle in enumerate(angles_resampled):
                if abs(angle - last_direction) < 0.25:  # Penalize repeated angles
                    ranges_resampled[i] = 0

        # Find the direction of the largest clear space
        largest_gap_index = np.argmax(ranges_resampled)
        direction = angles_resampled[largest_gap_index]

        rospy.loginfo(f"Clear direction found: {direction:.2f} radians")
        return direction


    def move_in_direction(self, laser_data, direction):
        """Move the robot in the specified direction while avoiding obstacles."""
        rate = rospy.Rate(10)  # 10 Hz
        stop_distance = 0.05  # Stop if obstacle is within 0.5 meters
        angular_correction = 0.005  # Adjust heading dynamically

        while not rospy.is_shutdown():
            # Convert laser scan ranges to a numpy array
            ranges = np.array(laser_data.ranges)
            max_range = laser_data.range_max
            angles = np.linspace(laser_data.angle_min, laser_data.angle_max, len(ranges))

            # Mask invalid ranges (0 or > max range)
            valid_ranges = np.where((ranges > 0.1) & (ranges < max_range), ranges, 0)

            # Check for obstacles in front and slightly to the sides (-30° to +30°)
            forward_mask = (angles > -np.pi / 6) & (angles < np.pi / 6)  # -30° to +30°
            forward_ranges = valid_ranges[forward_mask]

            if len(forward_ranges) > 0 and np.min(forward_ranges) < stop_distance:
                rospy.logwarn("Obstacle detected! Stopping and turning.")
                self.stop_robot()

                # Find the direction of the largest clear space
                direction = self.get_clear_direction(laser_data)
                rospy.loginfo(f"Turning to new direction: {direction:.2f} radians")
                self.turn_to_angle(direction)
                break  # Exit loop and re-evaluate

            # If no obstacles, continue moving in the given direction
            vel_msg = Twist()
            vel_msg.linear.x = 0.03  # Forward speed
            vel_msg.angular.z = angular_correction * 0.005  # Small adjustments to heading
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()



   
    def run(self):
        """Main navigation logic."""
        rate = rospy.Rate(2)  # 2 Hz loop rate
        error_threshold = 0.05  # Stop when the AR tag distance is within 0.5 meters

        while not rospy.is_shutdown():
            try:
                # Get the transform between AR markers
                transform = self.tf_buffer.lookup_transform("ar_marker_3", "ar_marker_4", rospy.Time(0))
                dx = transform.transform.translation.x
                dy = transform.transform.translation.y
                distance = np.sqrt(dx**2 + dy**2)
                rospy.loginfo(f"Distance between AR tags: {distance:.2f} meters")

                # Check if distance is within the error threshold
                if distance <= error_threshold:
                    rospy.loginfo("AR tags are close enough. Stopping navigation.")
                    self.stop_robot()
                    break

                # Get LaserScan data to detect empty space and obstacles
                laser_data = rospy.wait_for_message("/scan", LaserScan)
                direction = self.get_clear_direction(laser_data)
                rospy.loginfo(f"Moving in direction: {direction:.2f} radians")

                # Move in the direction while avoiding obstacles
                self.move_in_direction(laser_data, direction)

            except LookupException as e:
                rospy.logwarn(f"Transform lookup failed: {e}. Retrying...")
            except Exception as e:
                rospy.logerr(f"Unexpected error occurred: {e}")
                self.stop_robot()
                break

            rate.sleep()
    
    def set_initial_pose(self, x, y, theta):
        """Set the initial pose of the robot in the map frame."""
        # Create initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()

        # Set position
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0

        # Convert theta to radians if it's in degrees
        theta_radians = math.radians(theta) if abs(theta) > 2 * math.pi else theta

        # Set orientation (convert theta to quaternion)
        quat = quaternion_from_euler(0, 0, theta_radians)  # roll, pitch, yaw
        initial_pose.pose.pose.orientation.x = quat[0]
        initial_pose.pose.pose.orientation.y = quat[1]
        initial_pose.pose.pose.orientation.z = quat[2]
        initial_pose.pose.pose.orientation.w = quat[3]

        # Set covariance (optional, typically default values work)
        initial_pose.pose.covariance = [0.0] * 36

        # Publish initial pose
        rospy.loginfo(f"Publishing initial pose: x={x}, y={y}, theta={theta_radians}")
        self.initial_pose_pub.publish(initial_pose)
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        node = ARNavigationDiscrete()
        node.set_initial_pose(x=-0.589, y=-0.435, theta=0.816)
        node.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

def lidar_callback(msg):
    """
    Callback for LiDAR data.
    Publishes a basic occupancy grid.
    """
    grid = OccupancyGrid()
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = "map"
    # Basic grid construction (placeholder logic)
    grid.data = [0 if r < 1.0 else 100 for r in msg.ranges]
    lidar_pub.publish(grid)

if __name__ == "__main__":
    rospy.init_node("lidar")
    lidar_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.loginfo("LiDAR processing node running...")
    rospy.spin()

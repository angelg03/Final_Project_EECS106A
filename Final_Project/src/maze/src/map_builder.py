#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

ar_positions = []
occupancy_grid = None

def ar_pose_callback(msg):
    """
    Collect AR tag poses for mapping.
    """
    ar_positions.append((msg.pose.position.x, msg.pose.position.y))

def grid_callback(msg):
    """
    Update the occupancy grid from LiDAR data.
    """
    global occupancy_grid
    occupancy_grid = msg

def combine_data():
    """
    Combine AR tag and LiDAR data to build a map.
    """
    if occupancy_grid is None or not ar_positions:
        return
    
    # Modify the occupancy grid with AR tag positions (placeholder logic)
    rospy.loginfo("Building combined map...")
    combined_map = occupancy_grid
    # Publish combined map (placeholder for actual integration logic)
    combined_pub.publish(combined_map)

if __name__ == "__main__":
    rospy.init_node("map_builder")
    rospy.Subscriber("/ar_pose", PoseStamped, ar_pose_callback)
    rospy.Subscriber("/occupancy_grid", OccupancyGrid, grid_callback)
    combined_pub = rospy.Publisher("/combined_map", OccupancyGrid, queue_size=10)
    rospy.loginfo("Map builder node running...")
    rate = rospy.Rate(1)  # 1 Hz update rate
    while not rospy.is_shutdown():
        combine_data()
        rate.sleep()

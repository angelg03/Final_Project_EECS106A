#!/usr/bin/env python3

import rospy
import subprocess

if __name__ == "__main__":
    rospy.init_node("main_navigation")
    rospy.loginfo("Starting navigation system...")
    
    # Launch other nodes (ensure paths are correct)
    subprocess.Popen(["rosrun", "maze", "ar_processing.py"])
    subprocess.Popen(["rosrun", "maze", "lidar.py"])
    subprocess.Popen(["rosrun", "maze", "map_builder.py"])

    rospy.spin()

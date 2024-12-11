#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

def ar_callback(msg):
    """
    Callback for AR tag detection.
    Publishes the pose of detected AR tags.
    """
    for marker in msg.markers:
        rospy.loginfo(f"Detected AR tag: {marker.id}")
        ar_pose = PoseStamped()
        ar_pose.header = marker.header
        ar_pose.pose = marker.pose.pose
        ar_pub.publish(ar_pose)

if __name__ == "__main__":
    rospy.init_node("ar_processing")
    ar_pub = rospy.Publisher("/ar_pose", PoseStamped, queue_size=10)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)
    rospy.loginfo("AR processing node running...")
    rospy.spin()

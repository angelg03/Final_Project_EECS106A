#!/usr/bin/env python3
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
import math

def normalize_quaternion(q):
    """
    Normalize a quaternion to ensure its magnitude is 1.
    :param q: geometry_msgs/Quaternion
    :return: Normalized quaternion
    """
    magnitude = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    q.x /= magnitude
    q.y /= magnitude
    q.z /= magnitude
    q.w /= magnitude
    return q

def ar_callback(msg):
    """
    Callback to process AR tag detections and publish normalized poses.
    """
    for marker in msg.markers:
        rospy.loginfo(f"Detected AR tag: {marker.id}")
        
        # Normalize the quaternion
        pose = PoseStamped()
        pose.header = marker.header
        pose.pose = marker.pose.pose
        pose.pose.orientation = normalize_quaternion(pose.pose.orientation)
        
        # Publish the normalized pose
        ar_pose_pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("ar_processing")
    ar_pose_pub = rospy.Publisher("/ar_pose", PoseStamped, queue_size=10)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)
    rospy.spin()


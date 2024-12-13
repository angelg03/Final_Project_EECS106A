#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

class ARTagGoalPublisher:
    def __init__(self):
        rospy.init_node('ar_tag_goal_publisher', anonymous=True)
        self.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def marker_callback(self, msg):
        if not msg.markers:
            return
        marker = msg.markers[0]  # Assuming the first marker is the target
        camera_pose = PoseStamped()
        camera_pose.header = marker.header
        camera_pose.pose = marker.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform("map", camera_pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            map_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, transform)
            self.goal_pub.publish(map_pose)
            rospy.loginfo("Published AR tag goal to move_base.")
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: %s", str(e))

if __name__ == "__main__":
    node = ARTagGoalPublisher()
    rospy.spin()

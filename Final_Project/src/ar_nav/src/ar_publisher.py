#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

class ARTagPublisher:
    def __init__(self):
        rospy.init_node('ar_tag_publisher', anonymous=True)

        # Subscriber for AR tag detections
        self.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback, queue_size=1)

        # Publisher for detected AR tag poses
        self.tag_pose_pub = rospy.Publisher("/ar_tag_poses", PoseStamped, queue_size=10)

        rospy.loginfo("AR Tag Publisher initialized and listening for AR markers.")

    def marker_callback(self, msg):
        # Check if any markers are detected
        if not msg.markers:
            return

        # Publish poses for all detected markers
        for marker in msg.markers:
            tag_pose = PoseStamped()
            tag_pose.header = marker.header  # Maintain the frame from the detection source
            tag_pose.pose = marker.pose.pose  # Detected pose

            # Log the detected tag ID and pose
            #rospy.loginfo(f"Detected AR Tag {marker.id}: Pose = {tag_pose.pose}")

            # Publish the pose
            self.tag_pose_pub.publish(tag_pose)

if __name__ == "__main__":
    try:
        ARTagPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

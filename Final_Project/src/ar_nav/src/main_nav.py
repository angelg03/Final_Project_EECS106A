#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

class ARNavigationMain:
    def __init__(self):
        rospy.init_node('ar_navigation_main', anonymous=True)

        # Connect to the move_base action server
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        # Subscribe to pose goals (published when AR tag is detected and transformed)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)

        self.current_goal = None
        self.new_goal_received = False
        self.rate = rospy.Rate(2)  # Check twice per second

    def goal_callback(self, msg):
        self.current_goal = msg
        self.new_goal_received = True
        rospy.loginfo("New AR tag goal received.")

    def send_goal_to_move_base(self, pose_stamped):
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped

        rospy.loginfo("Sending goal to move_base...")
        self.move_base_client.send_goal(goal)
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(60.0))

        if not finished_within_time:
            self.move_base_client.cancel_goal()
            rospy.logwarn("Timed out achieving navigation goal.")
        else:
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully!")
            else:
                rospy.logwarn("Failed to reach goal. State: %d", state)

    def run(self):
        while not rospy.is_shutdown():
            if self.new_goal_received and self.current_goal is not None:
                self.new_goal_received = False
                self.send_goal_to_move_base(self.current_goal)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = ARNavigationMain()
        node.run()
    except rospy.ROSInterruptException:
        pass

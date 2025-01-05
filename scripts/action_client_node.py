#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import RobotState  # Custom message
from geometry_msgs.msg import Point  # To publish the last target

class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node')
        
        # Action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        rospy.loginfo("Action server connected!")

        # Publisher for robot state
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        self.robot_state = RobotState()

        # Publisher for last target
        self.last_target_pub = rospy.Publisher('/last_target', Point, queue_size=10)

        # Subscriber for odometry
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        self.robot_state.vel_x = msg.twist.twist.linear.x
        self.robot_state.vel_z = msg.twist.twist.angular.z
        self.state_pub.publish(self.robot_state)

    def send_goal(self, x, y):
        # Publish last target to the /last_target topic
        target_point = Point(x=x, y=y, z=0.0)
        self.last_target_pub.publish(target_point)

        # Send the goal to the action server
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        rospy.loginfo(f"Goal sent: ({x}, {y})")

    def feedback_callback(self, feedback):
        rospy.loginfo(f"Feedback received: {feedback}")

    def cancel_goal(self):
        self.client.cancel_goal()
        rospy.loginfo("Goal canceled")

    def run(self):
        rospy.loginfo("Action Client Node running...")
        while not rospy.is_shutdown():
            cmd = input("Enter 's' to send a goal, 'c' to cancel, or 'q' to quit: ")
            if cmd == 's':
                x = float(input("Enter target x: "))
                y = float(input("Enter target y: "))
                self.send_goal(x, y)
            elif cmd == 'c':
                self.cancel_goal()
            elif cmd == 'q':
                break

if __name__ == "__main__":
    node = ActionClientNode()
    node.run()

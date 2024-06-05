#!/usr/bin/env python

import rospy
import actionlib
from course_web_dev_ros.msg import WaypointActionAction, WaypointActionGoal
from geometry_msgs.msg import Point

class WaypointActionClient(object):
    def __init__(self):
        # Initialize the action client node
        rospy.init_node('waypoint_action_client')

        # Create the action client
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

        # Wait for the action server to come up
        rospy.loginfo("Waiting for action server to start.")
        self.client.wait_for_server()
        rospy.loginfo("Action server started, sending goal.")

    def send_goal(self, x, y, z):
        # Creates a goal to send to the action server
        goal = WaypointActionGoal()
        goal.position = Point(x, y, z)

        # Sends the goal to the action server, specifying callbacks
        self.client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

        # Waits for the server to finish performing the action
        self.client.wait_for_result()

        # Prints out the result of executing the action
        return self.client.get_result()

    def done_callback(self, state, result):
        rospy.loginfo("Action completed: %s" % str(result.success))
        rospy.loginfo("Action State: %s" % str(state))

    def feedback_callback(self, feedback):
        rospy.loginfo("Current position: %s" % str(feedback.position))
        rospy.loginfo("Current state: %s" % feedback.state)

if __name__ == '__main__':
    # Initializes the client and sends a goal
    client = WaypointActionClient()
    x = 2.0  # Example x-coordinate
    y = 2.0  # Example y-coordinate
    z = 0.0  # Example z-coordinate
    result = client.send_goal(x, y, z)
    rospy.loginfo("Result: %s" % str(result.success))

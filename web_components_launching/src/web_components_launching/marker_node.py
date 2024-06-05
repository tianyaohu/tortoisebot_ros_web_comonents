#!/usr/bin/env python3

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
import actionlib
from course_web_dev_ros.msg import WaypointActionAction, WaypointActionGoal
from geometry_msgs.msg import Point

class MarkerFactory:
    def __init__(self, server, frame_id="map"):
        """
        Initialize the MarkerFactory with a reference to an InteractiveMarkerServer,
        a default frame_id, and an action client for handling waypoints.
        """
        self.server = server
        self.frame_id = frame_id
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        rospy.loginfo("Waiting for action server to start.")
        self.client.wait_for_server()
        rospy.loginfo("Action server started, ready to send goals.")

    def make_interactive_marker(self, name, description, x, y, z):
        """
        Creates an interactive marker at the specified location and inserts it into the server.
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = name
        int_marker.description = description
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = z

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker_control = InteractiveMarkerControl()
        marker_control.interaction_mode = InteractiveMarkerControl.BUTTON
        marker_control.always_visible = True
        marker_control.markers.append(marker)
        int_marker.controls.append(marker_control)

        self.server.insert(int_marker, self.handle_feedback)
        self.server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(f"Marker {feedback.marker_name} clicked at {feedback.pose.position.x}, {feedback.pose.position.y}, {feedback.pose.position.z}")
            self.send_goal(feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z)

    def send_goal(self, x, y, z):
        goal = WaypointActionGoal()
        goal.position = Point(x, y, z)
        self.client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()
        return self.client.get_result()

    def done_callback(self, state, result):
        rospy.loginfo("Action completed: %s" % str(result.success))
        rospy.loginfo("Action State: %s" % str(state))

    def feedback_callback(self, feedback):
        rospy.loginfo("Current position: %s" % str(feedback.position))
        rospy.loginfo("Current state: %s" % feedback.state)

if __name__ == '__main__':
    rospy.init_node("simple_marker")
    server = InteractiveMarkerServer("simple_marker")
    marker_factory = MarkerFactory(server)

    # Create markers based on the specified waypoints
    marker_factory.make_interactive_marker("origin", "Initial Pos 0", 0.15, -0.48, 0)
    marker_factory.make_interactive_marker("waypoint1", "Waypoint 1", 0.58, -0.47, 0)
    marker_factory.make_interactive_marker("waypoint2", "Waypoint 2", 0.638, 0.457, 0)
    marker_factory.make_interactive_marker("waypoint3", "Waypoint 3", 0.242, 0.486, 0)
    marker_factory.make_interactive_marker("waypoint4", "Waypoint 4", 0.163, 0.107, 0)
    marker_factory.make_interactive_marker("waypoint5", "Waypoint 5", -0.1, 0, 0)
    
    # Fork RIGHT
    marker_factory.make_interactive_marker("waypoint6_right", "Waypoint 6 Right", -0.227, 0.458, 0)
    marker_factory.make_interactive_marker("waypoint7_right", "Waypoint 7 Right", -0.663, 0.451, 0)
    
    # Fork LEFT
    marker_factory.make_interactive_marker("waypoint6_left", "Waypoint 6 Left", -0.264, -0.471, 0)
    marker_factory.make_interactive_marker("waypoint7_left", "Waypoint 7 Left", -0.514, -0.462, 0)

    rospy.spin()

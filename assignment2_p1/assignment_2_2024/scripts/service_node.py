#!/usr/bin/env python3

import rospy
from assignment_2_2024.srv import GetTarget, GetTargetResponse
from geometry_msgs.msg import Point

# Global variable to store the last target
last_target = {"x": 0.0, "y": 0.0}

def handle_get_target(req):
    rospy.loginfo(f"Returning last target: ({last_target['x']}, {last_target['y']})")
    return GetTargetResponse(last_target["x"], last_target["y"])

def target_callback(msg):
    global last_target
    last_target["x"] = msg.x
    last_target["y"] = msg.y
    rospy.loginfo(f"Updated last target to: ({last_target['x']}, {last_target['y']})")

def main():
    rospy.init_node('service_node')

    # Subscribe to /last_target
    rospy.Subscriber('/last_target', Point, target_callback)

    # Advertise the service
    rospy.Service('get_target', GetTarget, handle_get_target)
    rospy.loginfo("Service 'get_target' is ready to provide the last target coordinates.")

    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_urdf.msg import RobotState  # Custom message


class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(RobotState, '/robot_state', 10)

        # Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Internal state
        self.robot_state = RobotState()

        # Log startup
        self.get_logger().info('MoveRobotNode with Odometry Subscriber has started!')

    def odom_callback(self, msg):
        """Callback for odometry data."""
        # Extract position and velocity
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        self.robot_state.vel_x = msg.twist.twist.linear.x
        self.robot_state.vel_z = msg.twist.twist.angular.z

        # Publish the custom RobotState message
        self.state_pub.publish(self.robot_state)


    def move_robot(self, linear, angular):
        """Publishes velocity commands."""
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f'Moving: Linear Velocity = {linear}, Angular Velocity = {angular}')

    def stop_robot(self):
        """Stops the robot by publishing zero velocities."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Robot stopped!')


def spin_node_in_thread(node):
    """Runs rclpy.spin in a separate thread."""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    move_robot_node = MoveRobotNode()

    # Run rclpy.spin in a separate thread
    spin_thread = Thread(target=spin_node_in_thread, args=(move_robot_node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            print("\nControl the robot:")
            print("1. Enter linear and angular velocities")
            print("2. Stop the robot")
            print("3. Quit")
            choice = input("Enter your choice (1/2/3): ")

            if choice == '1':
                try:
                    linear = float(input("Enter linear velocity (m/s): "))
                    angular = float(input("Enter angular velocity (rad/s): "))
                    move_robot_node.move_robot(linear, angular)
                except ValueError:
                    print("Invalid input. Please enter numeric values.")
            elif choice == '2':
                move_robot_node.stop_robot()
            elif choice == '3':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please select 1, 2, or 3.")
    except KeyboardInterrupt:
        move_robot_node.get_logger().info('Keyboard Interrupt (Ctrl-C) detected. Stopping robot...')
    finally:
        move_robot_node.stop_robot()
        move_robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

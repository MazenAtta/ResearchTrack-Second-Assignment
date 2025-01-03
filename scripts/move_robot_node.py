#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # Create a publisher for /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Log node startup
        self.get_logger().info('MoveRobotNode has started!')

    def move_robot(self, linear, angular):
        """Publishes velocity commands based on user input."""
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


def main(args=None):
    rclpy.init(args=args)
    move_robot_node = MoveRobotNode()

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

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # Create a publisher for /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Define the movement pattern (e.g., forward, rotate, stop)
        self.timer = self.create_timer(0.1, self.move_robot)
        self.movement_state = 0  # 0: Forward, 1: Rotate
        self.forward_time = 30  # Duration to move forward (in timer ticks)
        self.rotate_time = 20   # Duration to rotate (in timer ticks)
        self.timer_counter = 0

        # Log node startup
        self.get_logger().info('MoveRobotNode has started!')

    def move_robot(self):
        """Publishes velocity commands to move the robot."""
        twist_msg = Twist()

        if self.movement_state == 0:
            # Move forward
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
            self.get_logger().info('Moving Forward')
        elif self.movement_state == 1:
            # Rotate
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5
            self.get_logger().info('Rotating')

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist_msg)

        # Update timer and state
        self.timer_counter += 1
        if self.movement_state == 0 and self.timer_counter >= self.forward_time:
            self.movement_state = 1
            self.timer_counter = 0
        elif self.movement_state == 1 and self.timer_counter >= self.rotate_time:
            self.movement_state = 0
            self.timer_counter = 0

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
        rclpy.spin(move_robot_node)
    except KeyboardInterrupt:
        move_robot_node.get_logger().info('Keyboard Interrupt (Ctrl-C) detected. Stopping robot...')
    finally:
        move_robot_node.stop_robot()
        move_robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

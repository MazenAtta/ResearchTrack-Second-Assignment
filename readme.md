# ROS2 Assignment: Robot Controller

## Description
This project implements a robot controller in a ROS2 environment using Gazebo simulation. The controller is designed as a single node capable of moving the robot around based on user commands. The simulation environment is a flat Gazebo world where the robot is spawned at `(2, 2)` by default, provided by the `robot_urdf` repository.

---

## Node Overview

### **Move Robot Node**

#### Node Name: `move_robot_node`

**Responsibilities:**
- Publishes velocity commands to control the robot's movement.
- Accepts user input to define movement parameters (linear and angular velocities).
- Stops the robot when required.

**Published Topics:**
- `/cmd_vel` (Message `geometry_msgs/Twist`): Publishes velocity commands for the robot's movement.

**Subscribed Topics:**
- `/odom` (Message `nav_msgs/Odometry`): Subscribes to the robot's odometry data for real-time position tracking.

**Usage:**
```bash
ros2 run robot_urdf move_robot_node.py
```

---

## Launch File

**Launch File Name:** `gazebo.launch.py`

This launch file initializes the following:
- Gazebo simulation with the robot model.


**Usage:**
```bash
ros2 launch robot_urdf gazebo.launch.py
```

---

### Build the Workspace
```bash
cd (~/ros2_ws)
colcon build
source install/setup.bash
```

---

## Usage Instructions

1. **Launch the Simulation:**
   ```bash
   ros2 launch robot_urdf gazebo.launch.py
   ```

2. **Run the Move Robot Node:**
   ```bash
   ros2 run robot_urdf move_robot_node.py
   ```

3. **Control the Robot:**
   - Enter linear and angular velocities interactively to move the robot.
   - Stop the robot by entering zero velocities.

---

## Dependencies

Ensure the following dependencies are installed:
- ROS2 Foxy
- `nav_msgs`
- `geometry_msgs`
- `rclpy`

---

## Author
Mazen Atta

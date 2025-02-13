# MrRobot README

## Project Overview
MrRobot is a versatile robotic system designed for autonomous navigation, object recognition, and real-time environmental interaction. It integrates with ROS 2 for efficient control and Gazebo for high-fidelity simulations. While it utilizes ArUco markers for localization, it also supports other navigation techniques, making it adaptable for various applications.

## Features
- Autonomous navigation using ROS 2
- Obstacle detection and avoidance
- Object recognition and interaction
- Gazebo simulation environment
- Real-time camera feed processing
- Customizable movement and control parameters

## Installation

### Prerequisites
- ROS 2 installed
- Gazebo simulation environment
- OpenCV and ArUco libraries
- Python 3.x

#set up instructions

# Clone the repository
git clone <repo-url>
cd <repo-directory>

# Install dependencies
pip install -r requirements.txt

# Build the ROS 2 package
colcon build

# Source the workspace
source install/setup.bash
 
# Launch the Gazebo simulation
tmux new -s gazebo
ros2 launch <package_name> <launch_file>.launch.py

# Start the robot with all capabilities
ros2 run <package_name> <node_name>

#Configuration
Modify config.yaml to adjust parameters like marker size, obstacle avoidance thresholds, movement settings, and object recognition parameters.

#Contributing
Feel free to submit issues and pull requests for improvements. Ensure your code follows proper ROS 2 standards.




# ROS 2 Custom DWA Planner with TurtleBot3 and Gazebo

This repository contains a custom implementation of the **Dynamic Window Approach (DWA) local planner** for ROS 2, designed to work with TurtleBot3 in a simulated Gazebo environment. It includes a launch file to bring up the simulation, planner, and RViz2 visualization.

## 🧩 Features

- Custom DWA planner for local trajectory generation
- Works with simulated TurtleBot3 in Gazebo
- Integrated RViz2 configuration for visualization
- ROS 2 Humble-compatible

---

## ⚙️ Prerequisites

Ensure you have the following installed:

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- `colcon` and ROS 2 development tools

---

## 🚀 Installation Steps

### 1. Setup ROS 2 Workspace

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/bisalisunil/ros2_custom_dwa_planner.git
```

### 2. Install Dependencies

Install required ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3* \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions
```

### 3. Set TurtleBot3 Model

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

Or export it manually each time:

```bash
export TURTLEBOT3_MODEL=burger
```

---

## 🛠️ Build the Workspace

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_dwa_planner
source install/setup.bash
```

---

## 📡 Launch the Simulation

```bash
ros2 launch custom_dwa_planner dwa_planner.launch.py
```

This will:
- Launch the TurtleBot3 in a Gazebo world
- Start the custom DWA planner node
- Open RViz2 with a preconfigured layout

---

## 📁 Directory Structure

```
ros2_custom_dwa_planner/
├── launch/
│   └── dwa_planner.launch.py
├── rviz/
│   └── default.rviz
├── src/
│   └── dwa_planner_node.cpp / .py
├── CMakeLists.txt
└── package.xml
```

---

## 🧠 About the Custom DWA Planner

The custom DWA planner performs the following:
- Samples velocity space based on dynamic constraints
- Simulates possible trajectories for each velocity sample
- Scores trajectories based on distance to goal, obstacle proximity, and heading
- Publishes optimal `cmd_vel` to drive the robot

This is a basic version that can be extended with additional features like:
- Velocity smoothing
- Dynamic reconfiguration
- Visualization markers

---

## 🧪 Testing Tips

- Check terminal logs for planner status
- Use RViz2 to visualize obstacles and planned paths
- Modify trajectory cost functions in code for tuning

---

## 🧊 License

This project is licensed under the MIT License.

---

## 🤝 Contributions

Pull requests and issue reports are welcome!

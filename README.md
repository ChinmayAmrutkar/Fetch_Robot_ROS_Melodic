# 🦾 Fetch Robot Control Suite (ROS Melodic - Ubuntu 18.04)

This repository contains a ROS-based modular control system for the **Fetch robot** in simulation. It allows for:

- ✅ Navigation to a 2D goal using velocity control  
- ✅ Orienting the robot head to look at a 3D point  
- ✅ Full 6-DOF Cartesian control of the arm using MoveIt (with roll-pitch-yaw support)  
- ✅ Move robot arm to "home/tuck" position  

---

## 📁 Repository Structure

```
fetch_ws/
├── src/
│   └── fetch_position_controller/
│       ├── scripts/
│       │   ├── go_to_point.py           # Base velocity controller
│       │   ├── look_at_point.py         # Head gaze controller using PointHeadAction
│       │   └── moveit_arm_control.py    # 6-DOF arm pose + joint pose + tuck
│       ├── worlds/
│       │   └── random_objects.world     # Custom Gazebo world with blocks and cubes
│       ├── launch/
│       │   └── custom_simulation.launch # Launch Fetch with custom world
│       ├── package.xml
│       └── CMakeLists.txt
```

---

## 🧰 System Requirements

- Ubuntu 18.04  
- ROS Melodic  
- Gazebo 9  
- MoveIt for Fetch  
- Fetch simulation packages:

```bash
sudo apt install -y \
  ros-melodic-fetch-gazebo \
  ros-melodic-fetch-gazebo-demo \
  ros-melodic-fetch-moveit-config
```

---

## ✅ Setup Instructions

### 1. Create and Build the Workspace

```bash
mkdir -p ~/fetch_ws/src
cd ~/fetch_ws
catkin_make
echo "source ~/fetch_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone the Repository

```bash
cd ~/fetch_ws/src
git clone https://github.com/ChinmayAmrutkar/Fetch_Robot_ROS_Melodic.git fetch_position_controller
cd ~/fetch_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 Launching the System

### Step 1: Start Simulation with Custom Objects

```bash
roslaunch fetch_position_controller custom_simulation.launch
```

This loads:
- A custom Gazebo world (`random_objects.world`)
- Fetch robot with full sensors and controllers

---

### Step 2: Use Individual Controllers

#### ➤ 1. Base Navigation Controller

```bash
rosrun fetch_position_controller go_to_point.py
```

Prompts:
```
Enter goal x: 1.5
Enter goal y: 0.5
```

The robot will navigate using velocity control to the target pose.

#### ➤ 2. Head Gaze Controller

```bash
rosrun fetch_position_controller look_at_point.py
```

Prompts:
```
Enter point to look at (x, y, z):
```

Sends a gaze goal to the head controller using `PointHeadAction`.

#### ➤ 3. MoveIt Arm Controller

```bash
rosrun fetch_position_controller moveit_arm_control.py
```

You can select from:
```
Choose mode:
[1] Go to Cartesian pose (x, y, z, roll, pitch, yaw)
[2] Go to tucked (home) joint pose
```

- **Option 1**: Sends full 6-DOF Cartesian pose using roll-pitch-yaw (RPY) → quaternion  
- **Option 2**: Sends robot arm to a tucked joint configuration (Home Position)

---

## 📡 ROS Topics Used

| Node | Key Topics |
|------|------------|
| `go_to_point.py` | `/odom`, `/base_controller/command` |
| `look_at_point.py` | `/head_controller/point_head/goal` |
| `moveit_arm_control.py` | `/arm_controller/follow_joint_trajectory`, `/joint_states`, TF |

To get current end-effector pose:
```bash
rosrun tf tf_echo base_link wrist_roll_link
```

To see joint states:
```bash
rostopic echo /joint_states
```

---

## ✅ Completed Tasks

- [x] Go to arbitrary 2D base position  
- [x] Point head to 3D location  
- [x] Move arm to 6-DOF Cartesian target (x, y, z, RPY)  
- [x] Go to custom joint-space home/tucked pose  

---

## 🔜 Upcoming Tasks

- [ ] Gripper control (open/close fingers with force/position)  
- [ ] Pick and place pipeline using perception and arm + gripper  
- [ ] Object recognition and pose estimation for manipulation  
- [ ] Full pick-and-place demo with head, arm, and gripper coordination  

---

## 🧠 Author

**Chinmay Amrutkar**  
M.S. Robotics and Autonomous Systems – AI  
Arizona State University  
GitHub: [@ChinmayAmrutkar](https://github.com/ChinmayAmrutkar)

---

## 🏷️ License

This project is licensed under the **MIT License**

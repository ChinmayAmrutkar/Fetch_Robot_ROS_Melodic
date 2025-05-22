# 🦾 Fetch Robot Position Controller (ROS Melodic - Ubuntu 18.04)

This repository contains a ROS node that enables a **Fetch robot** (in simulation) to navigate to a given 2D goal `(x, y)` using velocity control. The robot estimates its position using `/odom` and applies a basic proportional controller to reach the target.

---

## 📦 Repository Structure

```
fetch_ws/
├── src/
│   └── fetch_position_controller/
│       ├── scripts/
│       │   └── go_to_point.py      # <-- Main position control node
│       ├── package.xml
│       └── CMakeLists.txt
```

---

## 🧰 System Requirements

- Ubuntu 18.04
- ROS Melodic
- Gazebo 9
- Fetch Simulation packages:
  - `ros-melodic-fetch-gazebo`
  - `ros-melodic-fetch-gazebo-demo`

---

## ✅ Setup Instructions

### 1. Install ROS Melodic

Follow the official guide: https://wiki.ros.org/melodic/Installation/Ubuntu

Then initialize:
```bash
sudo rosdep init
rosdep update
```

---

### 2. Install Gazebo + Fetch Packages

```bash
sudo apt install -y \
  ros-melodic-gazebo-ros \
  ros-melodic-fetch-gazebo \
  ros-melodic-fetch-gazebo-demo
```

---

### 3. Create and Build Your Workspace

```bash
mkdir -p ~/fetch_ws/src
cd ~/fetch_ws
catkin_make
echo "source ~/fetch_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 4. Clone or Create the Package

```bash
cd ~/fetch_ws/src
catkin_create_pkg fetch_position_controller rospy geometry_msgs nav_msgs
```

Add your ROS node under `scripts/` (already included in this repo):
- [`scripts/go_to_point.py`](scripts/go_to_point.py)

Make it executable:
```bash
chmod +x scripts/go_to_point.py
```

---

### 5. Build and Source the Workspace

```bash
cd ~/fetch_ws
catkin_make
source devel/setup.bash
```

---

### 6. Launch the Fetch Robot Simulation

```bash
roslaunch fetch_gazebo simulation.launch
```

Wait until Gazebo and the robot are fully initialized.

---

### 7. Run the Position Controller Node

```bash
rosrun fetch_position_controller go_to_point.py
```

The node will prompt you to enter a target position like:
```
Enter goal x: 1.5
Enter goal y: 0.5
```

The robot will compute its current position from `/odom` and navigate toward the goal using a velocity-based controller.

---

## 📡 Topics Used

| Topic | Type | Purpose |
|-------|------|---------|
| `/odom` | `nav_msgs/Odometry` | Robot's current position |
| `/base_controller/command` | `geometry_msgs/Twist` | Velocity commands to move the base |

---

## 🚀 Next Steps

This is Step 1 in a full series to master the Fetch robot:

- [x] Base position controller ✅
- [ ] Head movement (tilt and pan)
- [ ] Arm joint control
- [ ] Camera-based object detection
- [ ] Laser-based obstacle avoidance
- [ ] Pick-and-place with gripper

---

## 👤 Author

**Chinmay Amrutkar**  
MS Robotics and Autonomous Systems, ASU

---

## 🏷️ License

This project is licensed under the MIT License.

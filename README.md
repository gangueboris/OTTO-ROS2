```markdown
# OTTO Ground Control Station

A tactical, zero-latency Web Interface and ROS 2 backend for the bimodal OTTO robot. This project provides a complete Ground Control Station (GCS) to manage manual teleoperation, live camera streaming, Lifelong Mapping (SLAM), and autonomous navigation (Nav2) from a standard web browser.

## 🚀 Key Features

* **Bimodal Locomotion Control:** Seamlessly switch the chassis between Walk (gait-based) and Roll (diff-drive) modes via a custom Python state-machine.
* **Lifelong Mapping:** Integrates `slam_toolbox` in localization mode to dynamically expand the map when exploring unknown areas, updating the web interface in real-time.
* **Click-to-Navigate:** RVIZ-lite web integration using `ros2djs`. Tap anywhere on the generated map to dispatch autonomous Nav2 goals.
* **Hardware-Level E-Stop:** A custom 20Hz active software lock that exploits `twist_mux` priorities to instantly sever Nav2's control of the wheels during emergencies.
* **Picture-in-Picture (PiP) UI:** A responsive, Neo-Brutalist flexbox interface that dynamically routes MJPEG camera streams and canvas maps to save browser bandwidth and optimize screen real estate.

---

## 🏗️ System Architecture

### Frontend (Web Interface)
* **HTML/CSS/JS:** Vanilla web stack with a high-contrast tactical aesthetic.
* **roslibjs:** Handles websocket communication (Topics, Services, Actions) with the ROS 2 backend.
* **ros2djs & EaselJS:** Renders the SLAM occupancy grid and handles affine transformations for panning, zooming, and coordinate mapping.

### Backend (ROS 2)
* **rosbridge_server:** Bridges the ROS 2 network to the web browser via WebSockets.
* **otto_teleop.py:** The central locomotion engine. Handles web commands, manages the 20Hz E-Stop lock, and sequences walking trajectories.
* **twist_mux:** The hardware gatekeeper that prioritizes manual teleop commands over autonomous Nav2 commands.
* **slam_toolbox & Nav2:** Manages the `.posegraph` lifelong mapping and global/local path planning.

---

## 📦 Installation & Prerequisites

This project is built for **ROS 2** (Humble/Iron) and requires the following standard packages to be installed on the robot/host machine.

```bash
# Install core ROS 2 dependencies
sudo apt update
sudo apt install ros-$ROS_DISTRO-rosbridge-server \
                 ros-$ROS_DISTRO-slam-toolbox \
                 ros-$ROS_DISTRO-navigation2 \
                 ros-$ROS_DISTRO-nav2-bringup \
                 ros-$ROS_DISTRO-twist-mux \
                 ros-$ROS_DISTRO-ros2-control \
                 ros-$ROS_DISTRO-ros2-controllers

```

*Note: You must also have a camera node and `web_video_server` running to broadcast the MJPEG stream to the web interface.*

---

## ⚙️ Setup and Launch

**1. Build the Workspace**
Clone this repository into your ROS 2 workspace and build it:

```bash
cd ~/otto_ws
colcon build --packages-select otto_description
source install/setup.bash
ros2 launch otto_description simulation.launch.py

```

**2. Launch the Robot Stack**
Start the ROS 2 backend (which includes the robot state publisher, twist_mux, SLAM, and Nav2):

```bash
ros2 launch otto_description nav_slam.launch.py

```

**3. Launch the Web Bridge**
In a new terminal, open the websocket and video servers:

```bash
ros2 launch otto_description web_control.launch.py

```

**4. Open the Interface**
Simply open `index.html` in any modern web browser. (If running on a separate device, ensure the `IP_ADDRESS` variable in `app.js` is set to the robot's local IP).

---

## 🎮 Interface Guide

* **Walk Mode:** The interface defaults to Walk mode. The map is disabled, and the main screen displays the live camera feed. Use the D-Pad to trigger walking gaits.
* **Roll Mode:** Flipping the main toggle shifts the chassis into diff-drive mode. The UI enters Picture-in-Picture mode: the SLAM map takes the center screen, and the camera moves to the secondary display.
* **Autonomous Navigation:** In Roll mode, tap anywhere on the map to drop a waypoint. Nav2 will calculate a path and drive the robot.
* **The E-Stop:** Hitting the red STOP button triggers the hardware lock. It will instantly halt walking, manual rolling, or autonomous navigation. Tap the map to release the lock and resume autonomy.

---


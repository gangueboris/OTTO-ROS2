
## 0. Create a workspace
## 1. Create a new directory for your workspace along with a 'src' folder inside it
mkdir -p ~/ros2_ws/src

## 2. Navigate into the root of your new workspace
cd ~/ros2_ws

## 3. Build the empty workspace using colcon 
colcon build

## 4. Source the new setup file so your terminal recognizes the workspace
source install/setup.bash

## ROS 2 Jazzy: Beginner's Command Cheat Sheet

This guide covers the essential commands needed to navigate, build, and troubleshoot a ROS 2 workspace, with a specific focus on Python development.

## 1. Environment Setup
Before you can use ROS 2 commands or run your code, your terminal needs to know where ROS 2 and your specific workspace are located.

* **Source the main ROS 2 installation:**
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
    *(Note: If you added this to your `~/.bashrc` earlier, this happens automatically when you open a new terminal).*

* **Source your personal workspace (run this *inside* your `ros2_ws` folder after building):**
    ```bash
    source install/setup.bash
    ```

## 2. Workspaces and Building
In ROS 2, you write your code inside a "workspace" (usually a folder named `ros2_ws`). You must build your workspace for ROS 2 to recognize your code.

* **Build the entire workspace (Run from the root of your workspace, e.g., `~/ros2_ws`):**
    ```bash
    colcon build
    ```
* **Build with symlinks (Highly recommended for Python!):**
    ```bash
    colcon build --symlink-install
    ```
    *Why?* This allows you to edit your Python scripts and see the changes immediately the next time you run them, without having to run `colcon build` all over again.

## 3. Creating Packages
Code in ROS 2 is organized into packages. Since you are focusing on Python, you will use the `ament_python` build type.

* **Create a new Python package (Run inside your `src` folder):**
    ```bash
    ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy
    ```
    * `<package_name>`: The name of your package (e.g., `my_robot_controller`).
    * `--dependencies rclpy`: Automatically links the ROS 2 Python library (`rclpy`).

* **List all installed packages:**
    ```bash
    ros2 pkg list
    ```

## 4. Running Code
There are two main ways to start your code in ROS 2: running individual "nodes" (single scripts) or running "launch files" (which start multiple nodes at once).

* **Run a specific node:**
    ```bash
    ros2 run <package_name> <executable_name>
    ```
    *Example:* `ros2 run turtlesim turtlesim_node`

* **Run a launch file:**
    ```bash
    ros2 launch <package_name> <launch_file_name.launch.py>
    ```

## 5. System Introspection (Debugging)
These are your troubleshooting tools. When your robot isn't doing what you want, you will use these commands to see what is happening under the hood.

### Nodes (The active programs)
* **List all currently running nodes:**
    ```bash
    ros2 node list
    ```
* **Get detailed info about a specific node (what it's publishing/subscribing to):**
    ```bash
    ros2 node info /<node_name>
    ```

### Topics (The data highways)
Nodes communicate by sending messages over "topics". 
* **List all active topics:**
    ```bash
    ros2 topic list
    ```
* **See the actual data being sent on a topic in real-time:**
    ```bash
    ros2 topic echo /<topic_name>
    ```
    *Example:* `ros2 topic echo /cmd_vel` will show you the exact speed commands being sent to the robot.
* **Find out what type of message a topic uses:**
    ```bash
    ros2 topic info /<topic_name>
    ```
* **Manually publish data to a topic from the terminal:**
    ```bash
    ros2 topic pub /<topic_name> <message_type> "<data>"
    ```
    *Example:* `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}"`

## 1. Go to your native Linux home directory
cd ~

## 2. Create a new workspace folder, and a 'src' folder inside it
mkdir -p ~/ros2_ws/src

## 3. Move into the 'src' folder (where your packages will live)
cd ~/ros2_ws/src



# Riemannian Motion Policy (RMP) Controller
This repository implements Riemannian Motion Policies (RMPs) for the Franka Emika robot. RMPs enable modular motion generation, incorporating policies such as obstacle avoidance and joint limit avoidance. 
The theoretical foundation is based on the paper ["Riemannian Motion Policies" by Nathan D. Ratliff et al.](https://arxiv.org/abs/1801.02854)

## Primitive Configuration
The controller reads [src/config.yaml](src/config.yaml) at startup. Each primitive now supports an `enabled` switch and a weight term in its own section.

The added `force_controller` primitive is a joint-space RMP that uses external joint torque estimates. Its inertia is smooth and differentiable, with a sigmoid-shaped activation:

- Inertia is near zero when external torque magnitude is below `low_torque_threshold` (default: `1.0` Nm).
- Inertia increases with torque and approaches `max_inertia` near `high_torque_threshold`.
- `sigmoid_steepness` controls how sharp the transition is.
- `weight_force_controller` scales the primitive contribution in the global RMP sum.

Key config section:

```yaml
force_controller:
  enabled: true
  low_torque_threshold: 1.0
  high_torque_threshold: 5.0
  max_inertia: 5.0
  sigmoid_steepness: 10.0
  damping: 2.0
  weight_force_controller: 1.0
```

### Prerequisites
Ensure you have the following installed and configured:

- **ROS 2 Humble**
- **libfranka v0.13.0 or newer**
- **franka_ros2 v0.13.1 or newer**
- **Moveit (see below)**

For detailed setup instructions, please refer to the [Franka ROS2 FCI documentation](https://support.franka.de/docs/franka_ros2.html).

If MoveIt is not already installed, follow the following instructions:
### MoveIt
Install [Moveit Humble](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html).
Follow the installtion steps there.\
Here is also a copy of the command. * We name our workspace `franka_ros2_ws`, which is
different from [Moveit Humble](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html).
```
source /opt/ros/humble/setup.bash
```
```
sudo apt install python3-rosdep
```
```
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```
```
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```
```
sudo apt install python3-vcstool
```
```
mkdir -p ~/franka_ros2_ws/src
```
```
cd ~/franka_ros2_ws/src
git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
```
```
vcs import < moveit2_tutorials/moveit2_tutorials.repos
```
```
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```
```
cd ~/franka_ros2_ws
colcon build --mixin release
```
If you are short on computer memory or generally your build is struggling to complete on your computer, 
you can append the argument `--parallel-workers [num of workers]` to the colcon command above.
```
source install/setup.bash
```
---

## Installation RMP

### Step 1: Clone the RMP Repository
Clone this repository into the `src` directory of your `franka_ros2_ws` workspace:

```bash
cd ~/franka_ros2_ws/src
git clone https://github.com/acaviezel/Riemannian-Motion-Policies-Franka-Emika-Robot.git
```

---

### Step 2: Modify Configuration for RMP Controller
Add the following lines to your `controllers.yaml` file, located at `franka_ros2/franka_bringup/config/`:

```yaml
riemannian_motion_policy:
  type: riemannian_motion_policy/RiemannianMotionPolicy
```

---

### Step 3: Clone the Messages Package
Clone the `messages_fr3` package into the `src` directory of your workspace:

```bash
cd ~/franka_ros2_ws/src
git clone https://github.com/acaviezel/messages_fr3.git
```

---

### Step 4: Build the Workspace
Build either the specific package or the entire workspace:

- To build the **RMP controller** package:

  ```bash
  colcon build --packages-select cartesian_impedance_control --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- To build **all packages** in the `src` folder:

  ```bash
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

---

### Step 5: Update `.bashrc` File
To ensure your setup is sourced each time you open a terminal, add the following line to the end of your `.bashrc` file:

```bash
source ~/franka_ros2_ws/install/setup.sh
```

To edit the `.bashrc` file:

```bash
nano ~/.bashrc
```

After editing, reload the file:

```bash
source ~/.bashrc
```

---

## Additional Setup: Distance Calculator and MoveIt Scene Loader
Follow the installation instructions provided in the [motion_planning_mt repository](https://github.com/acaviezel/motion_planning).

---

## Launching the RMP Controller with MoveIt

### Step 1: Start the MoveIt Environment
Launch the MoveIt environment and the RMP controller:

```bash
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=<fci-ip>
```

Replace `<fci-ip>` with the IP address of your Franka Control Interface (FCI).

---

### Step 2: Launch the Scene Node
Run the scene node to populate the environment with three cylinders (adjustable as needed):

```bash
ros2 run motion_planning_mt cylinder_scene
```

---

### Step 3: Launch the Distance Calculator
Start the node to calculate the distance between each robot link and the nearest obstacle:

```bash
ros2 run motion_planning_mt distance_calculator
```

---

### Step 4: Visualize Obstacles and Minimum Distances in RViz
To visualize the obstacles and the minimum distance in RViz:

1. Open the **Displays** panel in RViz.
2. Click the "Add" button (bottom left corner).
3. Add the following:
   - **MarkerArray**: Set the topic to `/rviz_visual_tools`.
   - **MarkerArray**: Set the topic to `/minimum_distance_visualization`.

---


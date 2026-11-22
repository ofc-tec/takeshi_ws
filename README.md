# Takeshi ROS 2 Workspace

ROS 2 Jazzy workspace for the Toyota Human Support Robot (HSR), Takeshi.

Repository: https://github.com/ofc-tec/takeshi_ws

This repository contains our custom ROS 2 packages, launch files, and robot behaviors. The original Toyota HSR packages are maintained separately and are not tracked by this repository.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim
- MoveIt 2
- colcon

Install the ROS 2 development tools:

```bash
sudo apt update
sudo apt install \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-ros-gz
```

Initialize rosdep if necessary:

```bash
sudo rosdep init
rosdep update
```

## Workspace Structure

The repository is located directly inside `src`:

```text
~/takeshi_ws/
├── src/                        # Git repository root
│   ├── .git/
│   ├── .gitignore
│   ├── README.md
│   │
│   ├── takeshi_bringup/        # Our custom launch files
│   │
│   ├── hsrb_common/            # Toyota packages
│   ├── hsrb_controllers/
│   ├── hsrb_launch/
│   ├── hsrb_moveit/
│   ├── hsrb_simulator/
│   ├── ...
│   └── tmc_*/
│
├── build/
├── install/
└── log/
```

## Installation

### 1. Install ROS 2 Jazzy

Follow the official installation instructions:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Create the workspace

For a new installation:

```bash
mkdir -p ~/takeshi_ws/src
cd ~/takeshi_ws/src
git clone https://github.com/ofc-tec/takeshi_ws.git .
```

**Important:** The command above assumes `src` is empty.

For an existing Toyota HSR workspace, preserve the installed source repositories and initialize Git in `src` instead. Do not overwrite the existing packages.

### 3. Install Toyota HSR packages

Obtain the ROS 2 Jazzy versions of the Toyota HSR repositories from:

https://github.com/hsr-project

Follow Toyota's installation instructions to obtain the required repositories, dependencies, and simulator resources.

The Toyota packages must be available alongside our custom packages inside `~/takeshi_ws/src`.

**The Toyota repositories are intentionally excluded from our Git tracking.**

### 4. Install dependencies

```bash
cd ~/takeshi_ws

source /opt/ros/jazzy/setup.bash

rosdep install \
    --from-paths src \
    --ignore-src \
    -r -y \
    --rosdistro jazzy
```

### 5. Build

```bash
cd ~/takeshi_ws

colcon build --symlink-install
```

Source the workspace:

```bash
source ~/takeshi_ws/install/setup.bash
```

After modifying a custom package, it can be compiled individually:

```bash
cd ~/takeshi_ws

colcon build \
    --packages-select takeshi_bringup \
    --symlink-install
```

## Simulation

### Toyota HSR apartment

Launch the standard Toyota simulation:

```bash
ros2 launch hsrb_gazebo_launch \
    hsrb_apartment_world.launch.py
```

In another terminal:

```bash
source ~/takeshi_ws/install/setup.bash

ros2 launch hsrb_moveit_config \
    hsrb_demo.launch.py \
    use_sim_time:=true
```

This is our initial reference configuration for testing Takeshi's simulation, controllers, and MoveIt 2.

### RC2025 World

Our next integration will reuse the existing RC2025 Gazebo Sim environment developed for Festeshi.

The RC2025 world belongs to the `rto_description` package in the Robotino workspace.

The objective is to launch:

- RC2025 Gazebo world
- Toyota HSR simulation and controllers
- MoveIt 2

The corresponding launch file will be maintained in `takeshi_bringup`.

## Development

Our custom packages are tracked in this repository.

Toyota's original repositories remain independent, allowing upstream updates without mixing them with our own development.

Planned components include:

- `takeshi_bringup`: Simulation and robot launch files.
- Move Commander integration.
- Perception and OctoMap integration.
- Behavior trees for manipulation.
- Autonomous service robot tasks.

## License

Custom packages specify their respective licenses. Third-party packages retain their original licenses.

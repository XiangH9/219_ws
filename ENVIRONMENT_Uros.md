# Uros Environment Reproduction Guide

This document records the conda environment and ROS/Gazebo workspace setup used for the current `219_ws` project so that others can reproduce it quickly.

## Scope

- Workspace root: `/home/xiangh9/xhros2/219_ws`
- Conda environment: `Uros`
- ROS distro: `humble`
- Simulator: `Gazebo Classic` (`gzserver` / Gazebo 11 plugin path present)

## 1. Recreate the Conda Environment

The exported conda definition is stored at:

- [Uros.environment.yml](/home/xiangh9/xhros2/219_ws/Uros.environment.yml)

Create the environment with:

```bash
conda env create -f /home/xiangh9/xhros2/219_ws/Uros.environment.yml
conda activate Uros
```

Current exported environment summary:

- Env name: `Uros`
- Python: `3.10.20`
- Prefix: `/home/xiangh9/.conda/envs/Uros`

## 2. ROS and Gazebo Prerequisites

This workspace expects system ROS 2 Humble and Gazebo Classic to already be installed.

From the current machine state, the active runtime paths include:

```bash
PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
PATH=...:/home/xiangh9/.conda/envs/Uros/bin:...:/opt/ros/humble/bin:...
```

Recommended system packages, based on repo docs:

```bash
sudo apt-get install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
```

## 3. Build the Workspace

In a fresh shell:

```bash
conda activate Uros
source /opt/ros/humble/setup.bash
cd /home/xiangh9/xhros2/219_ws
colcon build --packages-up-to unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco --symlink-install --event-handlers console_direct+ --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install
```

If OCS2-related packages are needed too, use:

```bash
conda activate Uros
source /opt/ros/humble/setup.bash
cd /home/xiangh9/xhros2/219_ws
colcon build --packages-up-to ocs2_core unitree_guide_controller go1_description keyboard_input hardware_unitree_mujoco --symlink-install --event-handlers console_direct+ --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${HOME}/219_ws/install
```

## 4. Source the Built Workspace

After building:

```bash
conda activate Uros
source /opt/ros/humble/setup.bash
source /home/xiangh9/xhros2/219_ws/install/setup.bash
```

The current install tree contains packages such as:

- `unitree_guide_controller`
- `go1_description`
- `leg_pd_controller`
- `keyboard_input`
- `hardware_unitree_mujoco`
- `ocs2_core`
- `hpipm_catkin`
- `blasfeo_catkin`

## 5. Launch the Gazebo Classic Simulation

Standard launch:

```bash
conda activate Uros
source /opt/ros/humble/setup.bash
source /home/xiangh9/xhros2/219_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=go1_description
```

Debug/suspended launch:

```bash
conda activate Uros
source /opt/ros/humble/setup.bash
source /home/xiangh9/xhros2/219_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=go1_description debug:=true
```

## 6. Optional PlotJuggler Launch

```bash
conda activate Uros
source /opt/ros/humble/setup.bash
source /home/xiangh9/xhros2/219_ws/install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 QT_QPA_PLATFORM=xcb LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib ros2 run plotjuggler plotjuggler
```

## 7. Key Source Files Used by Gazebo Classic

The active robot/simulation description is assembled from:

- [robot.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/robot.xacro)
- [const.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/const.xacro)
- [leg.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/leg.xacro)
- [gazebo_classic.xacro](/home/xiangh9/xhros2/219_ws/src/quadruped_ros2_control-humble/descriptions/unitree/go1_description/xacro/gazebo_classic.xacro)

## 8. Quick Verification

After activation and sourcing, these checks should pass:

```bash
python --version
which ros2
which gzserver
echo $PYTHONPATH
echo $LD_LIBRARY_PATH
ros2 pkg list | rg "unitree_guide_controller|go1_description|leg_pd_controller"
```

Expected highlights:

- `python --version` -> `Python 3.10.20`
- `ros2` should resolve under `/opt/ros/humble/bin`
- `gzserver` should be available
- `unitree_guide_controller` should appear in `ros2 pkg list`

## 9. Notes

- The current machine had `base` active when queried globally, but the intended reproducible environment for this project is `Uros`.
- For clean reproduction, prefer always entering a fresh shell and running:

```bash
conda activate Uros
source /opt/ros/humble/setup.bash
source /home/xiangh9/xhros2/219_ws/install/setup.bash
```

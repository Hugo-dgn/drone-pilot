# Drone Autopilot

This project aims to create a drone with the following capabilities:

- Going through a window
- Going through a tunnel
- Scanning QR codes
- Picking up objects and delivering them

Currently, the drone is capable of autonomously going through a window.

Your provided Markdown appears to be well-formed. However, I've made some minor adjustments for better readability:

## Install PX Toolchain

Ensure that you have a clean installation of Ubuntu 20 on your computer.

Look for updates and install Git if not already done:
```bash
sudo apt update
sudo apt upgrade
sudo apt install git
```

Clone the repository into your home directory:
```bash
cd ~
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git --recursive
```

Run the setup file:
```bash
cd Firmware
bash ./Tools/setup/ubuntu.sh
```

Build PX4 SITL (Software-in-the-Loop) for Gazebo:
```bash
cd src/Firmware
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo
```

## Create ROS Project

To initiate the ROS project with the necessary dependencies, use the following commands:

```bash
wget https://raw.githubusercontent.com/Hugo-dgn/drone-pilot/master/ubuntu_sim_ros_noetic.sh
bash ubuntu_sim_ros_noetic.sh pilot
rm ubuntu_sim_ros_noetic.sh
```

This will generate a ROS project named `pilot` in your current directory.

## Clone Repository

Clone the repository into the `src` folder within your ROS project:

```bash
cd pilot/src
git init
git remote add origin https://github.com/Hugo-dgn/drone-pilot
git pull origin master
cd ..
source devel/setup.bash
catkin build
```

## Install Dependencies

To install the dependencies, run the following command:

```bash
pip install -r requirements.txt
```

## Setup Parameters

In the file `pilot/src/parameters/params`, you must change this line:

```yaml
path_front_camera_calibration: "~/Documents/droneload/pilot/src/front_camera.yaml"
```

to:

```yaml
path_front_camera_calibration: "path_to_pilot/pilot/src/front_camera.yaml"
```

where `path_to_pilot` is the path to the folder where you have set up the ROS project.

## Launch the Gazebo Simulation

First, source all the dependencies:

```bash
source src/rossetup.bash
source devel/setup.bash
source src/gzsetup.bash
```

Then, launch the simulation:

```bash
roslaunch pilot main.launch
```

## Control the Drone

Use the `ground.py` file to control the drone.

To switch flight mode:

```bash
python ground.py switch n
```

Here, `n` is the number representing the flight mode:
- `0`: Altitude flight mode
- `1`: Offboard flight mode to go through the window

To choose which window to go through:

```bash
python ground.py window id
```

Here, `id` is the ID of the target window. This ID is displayed on the top right corner of the window on the camera feed.
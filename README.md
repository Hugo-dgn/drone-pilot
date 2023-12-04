# Droneload

## Create ROS Project

To initiate the ROS project with the necessary dependencies, use the following commands:

```bash
wget https://github.com/Hugo-dgn/drone-pilot/blob/master/ubuntu_sim_ros_noetic.sh
bash ubuntu_sim_ros_noetic.sh pilot
```

This will generate a ROS project named "pilot" in your current directory.

## Clone Repository:

Clone the repository into the `src` folder within your ROS project:

```bash
cd pilot/src
git init
git remote add origin https://github.com/Hugo-dgn/drone-pilot
git pull
catkin build
```

## Launch the gazebo simulation

To launch the simulation:

```bash
cd ..
source devel/setup.bash
roslaunch pilot main.launch
```

## Control the drone

With the file `ground.py`, you can control the drone.

To switch flight mode:

```bash
python ground.py switch n
```

Where `n` is the number representing the flight mode:
- 0: Altitude flight mode
- 1: Offboard flight mode to go through the window

To choose which window to go through:

```bash
python ground.py window id
```

Where `id` is the ID of the target window. This ID is displayed on the top right corner of the window on the camera feed back.
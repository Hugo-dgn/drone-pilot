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
cd pilot
git init
git git remote add origin https://github.com/Hugo-dgn/drone-pilot
git pull
```

## Launch the gazebo simulation

To launch the simulation :

```
roslaunch pilot main.launch
```
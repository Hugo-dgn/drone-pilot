path_to_firmware=~/src/Firmware

source $path_to_firmware/Tools/simulation/gazebo-classic/setup_gazebo.bash $path_to_firmware $path_to_firmware/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$path_to_firmware:$path_to_firmware/Tools/simulation/gazebo-classic/sitl_gazebo-classic
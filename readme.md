# Purpose
This package enables the control of an autonomous aerial vehicle (UAV) that uses [pixhawk](https://pixhawk.org/) hardware as a flight control unit (FCU).
It has been developed with the [PX4](https://px4.io/) flight stack in mind.
The original purpose was for simulating insect inspired behaviours, traditional waypoints can be combined with more complex "behaviours". 
It is intended for use with projects that make use of a companion computer. Some key features are:
* Extends PX4 ROS/GAZEBO simulation or can be run on a companion computer.
* Built using ROS middleware - therefore provides an easy interfaces to other robotics packages.
* No need to modify code in the FCU - therefore no risk of affecting the target vehicle's stability and no need to rebuild Firmware for each update.
* Built with python  

# Installation (for Ubuntu OS)
## Dependencies
* Install the PX4 toolchain https://dev.px4.io/master/en/setup/dev_env.html
* (if not achieved in the step above or otherwise) Install ROS supported by your operating system (must be ROS Kinetic or later) http://wiki.ros.org/ROS/Installation
* (recommended)  Install catkin tools: https://catkin-tools.readthedocs.io/en/latest/installing.html
* Install MAVROS - following the instructions (install as a debian package if possible) @ https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html


## Setting up the ROS workspace

If you want to setup a new ROS workspace for this project (recommended) then follow the steps at http://wiki.ros.org/catkin/Tutorials/create_a_workspace. 
An adapted example of this process is outlined in this section (change "kinetic" to your ROS installation).

First, open a terminal and (if not already done) source the main ROS installation 

```
source /opt/ros/kinetic/setup.bash
```
Now create an area for your installation, for example:
```
$ mkdir -p ~/ros_workspaces/pyx4_ws/src
```
Change directories to your workspace and clone the pyx4 repo, for example:
```
$ cd ~/ros_workspaces/pyx4_ws/src
$ git clone https://github.com/jannsta1/pyx4
```

Change directories to your workspace and build.
```
$ cd ~/ros_workspaces/pyx4_ws/
$ catkin build
```


## Environmental variables
A number of environmental variables are required to enable all of the software modules to communicate with each other. 
The snippet below shows what is included in a typical .bashrc file or favoured way of setting environmental variables (some of these are already described in the PX4 toolchain installation so careful not to duplicate environmental variables).  
```
# environmental variables used by pyx4 - adapt to your needs
export PX4_SRC_DIR=~/px4_myfork/Firmware    # point this to where your PX4 firmware is installed
export ROBOT_VRPN_NAME="SITL"               # only required if using the VRPN ROS package (Vicon)
export ROBOT_TYPE="SITL"                    # SITL / REAL
export ROBOT_STATE_ESTIMATION="SITL"        # GPS / VICON / VISION
export FCU_URL="127.0.0.1"
export GCS_URL="127.0.0.1"                  # or "udp://:14570@localhost:14550"

# adds the gazebo libraries (adapt to where you installed gazebo)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/worlds:/usr/share/gazebo-7

# function to set ROS/PX4 environmental variables
function setpx4() {

    source /opt/ros/kinetic/setup.bash       # point this to your main ROS installation

    current_dir="$PWD"
    build_ref=px4_sitl_default               
    
    cd ~/ros_workspaces/pixhawk_ws           # point this to your ROS pyx4 project
    source devel/setup.bash

    cd ${PX4_SRC_DIR}

    # PX4
    source ${PX4_SRC_DIR}/Tools/setup_gazebo.bash ${PX4_SRC_DIR} ${PX4_SRC_DIR}/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_SRC_DIR}:${PX4_SRC_DIR}/Tools/sitl_gazebo

    # tidy up
    cd "$current_dir"
}
setpx4                       # comment out if you don't want setpx4 to run automatically

```

# First steps
## Testing installation
To ensure that the setup procedure has been succesful. Run the following command in a terminal that has the the ros workspace loaded:
```
$ roslaunch pyx4 simple_mission.launch world:=empty.world
```
The default aircraft should takeoff and then follow some positional setpoints before returning to the origin and landing.


## Using .csv mission description
There are multiple ways to specify missions in pyx4. 
A simple way to get up and running is to use the CSV mission loader approach.
Using the templates found in pyx4_base/data/mission_specs, a sequence of position or velocity waypoints can be set for each control axis.

Once a mission has been specified in this fashion, it can be performed using the associated roslaunch file:
```
roslaunch pyx4 csv_mission.launch csv:=YOUR_MISSION_FILE.csv
``` 

- instruction_args can be passed. This is useful for sending unspecified parameters to a custom flight state. 
The arguments should be formatted like a dictionary but with semicolons to divide entries e.g. {speed:2 ; z_tgt:3.0}

# Concepts
todo  
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
## Install Dependencies

1. Install ROS Melodic (desktop full version): [http://wiki.ros.org/melodic/Installation](http://wiki.ros.org/melodic/Installation)
2. Install the PX4 Firmware: [https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html#sim_nuttx](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html#sim_nuttx)
3. Build the PX4 Firmware: [https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html#launching-gazebo-with-ros-wrappers](https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html#launching-gazebo-with-ros-wrappers)
4. Install catkin tools: [https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get)
5. Install mavros and mavros extras: [https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html#installation](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html#installation)


## Setting up the ROS workspace

If you want to setup a new ROS workspace for this project (recommended) then follow the steps at http://wiki.ros.org/catkin/Tutorials/create_a_workspace. 
An adapted example of this process is outlined in this section (change "kinetic" to your ROS installation).

First, open a terminal and (if not already done) source the main ROS installation 

```
source /opt/ros/{{ros-version}}/setup.bash
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
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/worlds:/usr/share/gazebo-7  # Change to gazebo 9 if applicable

# function to set ROS/PX4 environmental variables
function setpx4() {

    source /opt/ros/kinetic/setup.bash       # point this to your main ROS installation

    current_dir="$PWD"
    build_ref=px4_sitl_default               
    
    cd ~/ros_workspaces/pixhawk_ws           # point this to your ROS pyx4 project workspace
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
## Troubleshooting

#### ` [Err] [REST.cc:205] Error in REST request`

```
nano .ignition/fuel/config.yaml
```

- Change `url: https://api.ignitionfuel.org` to `url: https://api.ignitionrobotics.org`

- Run `source ~/.bashrc`

#### `[Err] [Server.cc:379] Could not open file[empty.world]` when running with `world:=empty.world`

- Check if `/home/user/path-to-Firmware/Firmware/Tools/sitl_gazebo/worlds` exists.
- If so, find the line `export GAZEBO_RESOURCE_PATH` and add: `~/Firmware/Tools/sitl_gazebo/worlds`
- Run `source ~/.bashrc`


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

# Testing
 
The goal of the testing platform is to ensure that no bugs are introduced when the code is modified.

For that, a test mission is created, which aim is to perform the basic logic of the codebase. Then, to test, the test_mission is performed and the testing platform checks that the position of the drone at each waypoint is the same as it was before the code was modified.

## Test mission: `data/mission_specs/basic_test.csv`

A CSV mission designed to test the basic logic of the pyx4 system.

1. To test *z* alone in target position: take off to 3m maintaining the x and y positions at the origin.
2. Test *x* alone in target position: advance 10m in *x*, maintain *y* and *z*.
3. Test *z* alone in target position: advance 10m in *x*, maintain *x* and *z*.
4. Test moving in *x*, *y* and *z* in target position: go back to the origin in *x* and *y* while increasing the altitude to 10m.
5. Test *x* alone in target velocity: advance in *x* at velocity 1 for 10s, maintain *y* and *z*.
6. Test *y* alone in target velocity: advance in *y* at velocity 1 for 10s, maintain *x* and *z*.
7. Test *x* and *y* in target velocity while moving *z* in target position: advance in *x* and *y* at velocity -2 for 12s while decreasing the altitude to 5m.
8. Test holding the position: maintain *x*, *y* and *z* for 5s.
9. Test *x* and *y* in target velocity while moving *z* in target position and adding yaw: advance in *x* and *y* at velocity 2 for 10s while increasing the altitude to 8m and adding a target yaw of 1.
10. Test landing: land at the starting point.

## Testing workflow

### Important files:
    - `pyx4_base/data/mission_specs/basic_test.csv`: the default test mission
    - `pyx4_base/data/test/basic_test.csv`: the test mission comparison file. For each waypoint, it contains the local positions the drone should be at when it reaches the waypoints.
    - `pyx4_base/test/pyx4.test`: launch the Px4 and Pyx4 nodes to perform the mission. Launch `pyx4_test_logic.py` to perform the test logics. Launch `pyx4_test.py` to perform the unittests.
    - `pyx4_base/get_test_data.py` *(optional)*: to get the comparisson file for testing. Alternatively, the waypoints could be computed by hand.
    - `pyx4_base/launch/get_test_data.launch`: launch file to launch `get_test_data.py`. 
    - `pyx4_base/pyx4_test_logic.py`: ROS node where most of the testing logic happens.
    - `pyx4_base/pyx4_test.py`: unittest node that performs the test assertions.
    
### Steps

#### Collecting data:

To test a comparison file is needed. This MUST be done when the code has passed some tests or when the developer is confident the code base is working properly.

If done manually, check the test mission to compute the final position for each waypoint and write a CSV in `pyx4_base/data/test` with the following columns:

- label: the waypoint number
- x: *x* position at the waypoint
- y: *y* position at the waypoint
- z: *z* position at the waypoint
- yaw: *yaw* position at the waypoint

If done automatically, run
```
roslaunch pyx4 get_test_data.launch mission:=yourmission.csv comp:=yourcomp.csv overwrite:=true/false
```
where
 - `mission` is the test mission. Default: `pyx4_base/data/mission_specs/basic_test.csv`;
 - `comp` is the comparison mission to write. Default: `pyx4_base/data/test/basic_test.csv`. 
 - `overwrite` is set to `false` by default. If set to true, it will overwrite the comp_file if it already exists. If `false` and the comparison file exists, it throws an exception.
 
#### Testing

Run 
```
rostest pyx4 pyx4.test mission:=yourmission.csv comp:=yourcomp.csv
```
where
- `mission` is, by default, `pyx4_base/data/mission_specs/basic_test.csv`, and
- `comp` is `pyx4_base/data/test/basic_test.csv`


# Teleoperation

Node to teleoperate the drone using keyboard inputs.

## Dependencies

Uses the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) node to take keyboard input. 

To install: 
```
sudo apt-get install ros-{{distro}}-teleop-twist-keyboard
```

## Usage

```
roslaunch pyx4 teleop.launch
```

And to use keyboard input:
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop
```

## TODO
- Better console output
- Fix coordinate frame?

# Concepts
todo  

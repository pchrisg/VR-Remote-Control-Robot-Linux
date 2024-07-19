# VR-Remote-Control-Robot-Linux

This code was only built on **Ubuntu Focal (20.04)** using **ROS Noetic Ninjemys**
The robot we use is a **CB3 UR5 robot**.

## Prerequisites
* Git
```
# To install git
sudo apt install git
```

* ROS
*https://wiki.ros.org/noetic/Installation/Ubuntu*
Follow the instructions to install ROS

* Create a ROS Workspace
*https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment*
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```


## Setting up the simulated robot
*https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md*

* Virtualbox
```
# To install virtualbox
sudo apt install virtualbox
sudo apt-get install virtualbox-guest-utils virtualbox-guest-x11 virtualbox-guest-dkms
```

* URSim
*https://www.universal-robots.com/download/software-cb-series/simulator-non-linux/offline-simulator-cb-series-non-linux-ursim-3158/*

  - Download URSim

  - Unpack the zipped virtual machine in a desired folder on your pc
  ```
  sudo apt-get install unrar
  cd ~/Downloads/
  mkdir ~/Documents/URSim_3.15.8.106339
  unrar x URSim_VIRTUAL-3.15.8.106339.rar ~/Documents/URSim_3.15.8.106339
  ```
  - Start VirtualBox and press 'New'

  - Define name (URSim_3.15.8), Type: Linux, Version: Ubuntu (64-bit)

  - Select Memory size of 768 MB and press 'Next'

  - Select Use an existing hard drive file and define the path to the folder where the zipped file was unpacked (~/Documents/URSim_3.15.8.106339), press 'Create'

  - Press 'Start' for starting the virtual machine

  - If an error saying 'Hardware acceleration is not available' is shown then it may be required to reboot the Windows computer into BIOS setup and enable hardware access to Virtual Machines and then restart Windows, VirtualBox and the virtual machine.


* Setting up a network connection

Next, setup a host network that will be used for the machine.

  - In Virtualbox, click File > Host Network Manager, press 'Create'

  - It should automatically get the static IP address 192.168.56.1. If not, set it up manually in the dialog. Close the window.

  - In the main window, select the URSim_3.15.8 machine and click on Settings.

  - On the left, select the Network point

  - Activate a network adapter, setting the 'Attached to' dropdown menu to Host-only Adapter

  - Select the network you just created, Click on the 'ok' button to close the window.


* Installing the External Control URCap
  - Download the latest **externalcontrol-x.x.x.urcap** from *https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases*

  - Create a Virtualbox share folder
  ```
  mkdir ~/VboxShare
  ```
  - Move the **externalcontrol-x.x.x.urcap** to the Virtalbox share folder

  - Make sure the virtual machine is off. Then select it and go to settings.

  - Select 'Share Folders' from the left. Add a folder on the right. Set the path to the share fodler you created. Make sure not to select 'Read-only', Select 'Auto-mount' and leave 'Mount point' blank.

  - Start the Virtual machine. Under Devices click 'Insert Guest Additions CD image...' and download from the internet when prompted.

  - Open terminal in the virtual machine 'Start' -> 'System Tools' -> 'UXTerm'.

  - Create a folder and mount the sharebox
  ```
  mkdir ShareFolder
  sudo mount -t vboxsf VboxShare ShareFolder
  ```

  - Open the folder 'Start' -> 'Accessories' -> 'FileManager PCManFM'. open ShareFolder.

  - copy the **externalcontrol-x.x.x.urcap** file to the 'Programs UR5' folder on the desktop.


  - Start the URSim UR5 program and follow the instructions at *https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md*


## Required Packages
* ROS-TCP-Endpoint
*https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/993d366b8900bf9f3d2da444fde64c0379b4dc7c*
```
cd ~/catkin_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

```

* MoveIt
*https://moveit.ros.org/install/source/*
```
# make sure ROS is updated
rosdep update
sudo apt update
sudo apt dist-upgrade

# install dependencies
sudo apt install python3-wstool python3-catkin-tools python3-rosdep


# install MoveIt
cd ~/catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/moveit/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```

* Universal Robot ROS Driver
*https://github.com/UniversalRobots/Universal_Robots_ROS_Driver*

Some changes were made to the files:
```
cd ~/catkin_ws/src
git clone https://github.com/pchrisg/Universal_Robots_Ros_Driver.git
```

* robotiq
*https://github.com/TAMS-Group/robotiq*

Some changes were made to the files:
```
cd ~/catkin_ws/src
git clone https://github.com/pchrisg/robotiq.git
```

* Before we pull the last repo, we must make sure we have all the dependencies for the other packages
```
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src/ --ignore-src --rosdistro noetic
```

* Now we clone our repository
```
cd ~/catkin_ws/src
git clone https://github.com/pchrisg/VR-Remote-Control-Robot-Linux.git
```

### Build the catkin workspace

* TO DO
  - Verify which packages are needed

```
cd ~/catkin_ws

# disable All High-Level User Interfaces (optional)
catkin config --blacklist \
    moveit_commander \
    #moveit_setup_assistant \
    moveit_ros_robot_interaction \
    moveit_ros_visualization \
    moveit_ros_benchmarks \
    moveit_controller_manager_example \
    moveit_ros_warehouse \
    moveit_ros_manipulation \
    moveit_visual_tools \
    rviz_visual_tools \

    # disable CHOMP Motion Planner (optional)
    moveit_chomp_optimizer_adapter \
    moveit_planners_chomp \
    chomp_motion_planner

catkin_make
```

# Run
**Run in 5 different terminals**

## Chris UR5 Moveit
* Universal_Robots_Ros_Driver
```
roslaunch chris_ur_launch ur5.launch
```
* Moveit controller
This can either be started in 1 terminal with 1 launch file
```
roslaunch chris_ur5_robotiq_config chris_move_group.launch
```
OR in 2 terminals with 2 launch files
```
roslaunch chris_ur5_robotiq_config movegroup.launch namespace:=ur5
roslaunch chris_ur5_robotiq_config movegroup.launch namespace:=feedback
```
* ROS-TCP-Endpoint and Movegroups
This can either be started in 1 terminal with 1 launch file
```
roslaunch chris_ur5_moveit chris_ur5_with_feedback.launch
```
OR in 2 terminals with 2 launch files
```
roslaunch chris_ur5_moveit chris_ur5_moveit.launch
roslaunch chris_ur5_moveit chris_robot_feedback.launch
```
## robotiq control
* robotiq simulation
```
roslaunch robotiq_gazebo robotiq.launch
```
* joint state publisher
```
rosrun robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_joint_states _model:=gazebo
```

## Useful ROS Commands

```
roscore
rosnode list
rosnode info </nodename>
rosnode ping <nodename>
rosrun [package_name] [node_name]

# debugging
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
rosrun rqt_graph rqt_graph


# ROS Topics
rostopic list -v
rostopic type <topicname>
rosmsg show <topicname>


# ROS Service
rosservice list 
rosservice call
rosservice type
rosservice find
rosservice uri

rosservice type /spawn | rossrv show


# ROS Params
rosparam set    set parameter
rosparam get    get parameter
rosparam load   load parameters from file
rosparam dump   dump parameters to file
rosparam delete delete parameter
rosparam list   list parameter names
```


## Creating a moveit config file
```
# Create URDF from .xacro
rosrun xacro xacro [name].xacro > [name].urdf

# Run Moveit Setup Assistant (https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html)
roslaunch moveit_setup_assistant setup_assistant.launch
```


# Git Token
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
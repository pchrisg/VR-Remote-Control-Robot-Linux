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


## Universal Robot ROS DRIVER
*https://github.com/UniversalRobots/Universal_Robots_ROS_Driver*

```
# clone the driver
cd ~/catkin_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the description. Currently, it is necessary to use the melodic-devel branch.
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make

# activate the workspace (ie: source it)
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


### Testing Universal Robot ROS Driver

In 3 separate windows run:
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.56.101

roslaunch ur5_moveit_config moveit_planning_execution.launch

roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
```


## Calibrating the robot
*https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_calibration/README.md*

```
# Replace your actual catkin_ws folder
cd ~/catkin_ws/src
catkin_create_pkg chris_ur_launch ur_client_library -D "Package containing calibrations and launch files for our UR robots."

# Create a skeleton package
mkdir -p chris_ur_launch/etc
mkdir -p chris_ur_launch/launch
```

Create the calibration file
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.56.101 target_filename:="$(rospack find chris_ur_launch)/etc/ur5_calibration.yaml"
```

Create a launchfile
```
cd ~/catkin_ws/src/chris_ur_launch/launch
roscp ur_robot_driver ur5_bringup.launch ur5.launch
```

And Update the .lanch file
```
<!-- Note: Only the relevant lines are printed here-->
  <arg name="robot_ip" default="192.168.0.101"/>
  <arg name="kinematics_config" default="$(find chris_ur_launch)/etc/ur5_calibration.yaml"/>
```


## Required Packages
* ROS-TCP-Endpoint
*https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/993d366b8900bf9f3d2da444fde64c0379b4dc7c*
```
cd ~/catkin_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

```

* MoveIt
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

# disable All High-Level User Interfaces (optional)
catkin config --blacklist \
    moveit_commander \
    moveit_setup_assistant \
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
```

* robotiq
*https://github.com/TAMS-Group/robotiq*
```
cd ~/catkin_ws/src
git clone https://github.com/pchrisg/robotiq.git

git clone https://github.com/pchrisg/Universal_Robots_Ros_Driver.git
```

#########################
# ROS Unity Integration #
#########################
https://github.com/Unity-Technologies/Unity-Robotics-Hub

roslaunch ros_tcp_endpoint endpoint.launch

#######################
# Unity Robotics Demo #
#######################
rosrun unity_robotics_demo color_publisher.py
rosrun unity_robotics_demo position_service.py

####################
# Chris UR5 Moveit #
####################
roslaunch chris_ur_launch ur5.launch
# roslaunch ur5_moveit_config moveit_planning_execution.launch
roslaunch chris_ur5_robotiq_config move_group.launch
roslaunch chris_ur5_moveit chris_ur5_moveit.launch

~~
roslaunch robotiq_gazebo robotiq.launch
rosrun robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_joint_states _model:=gazebo


####################
# Catkin WorkSpace #
####################
cd ~/catkin_ws/src
rosdep install -y --from-proslaunch ur5_moveit_config ur5_moveit_planning_execution.launchaths . --ignore-src --rosdistro noetic

#######
# ROS #
#######
roscore
rosnode list
rosnode info </nodename>
rosnode ping <nodename>
rosrun [package_name] [node_name]

#############
# ROS debug #
#############
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level


#########################################
# ROS Topics + ROS Service + ROS Params #
#########################################
rostopic list -v
rostopic type <topicname>
rosmsg show <topicname>

rosservice list 
rosservice call
rosservice type
rosservice find
rosservice uri

rosservice type /spawn | rossrv show


rosparam set    set parameter
rosparam get    get parameter
rosparam load   load parameters from file
rosparam dump   dump parameters to file
rosparam delete delete parameter
rosparam list   list parameter names

#################################
# Creating a moveit config file #
#################################

~Create URDF from .xacro
rosrun xacro xacro [name].xacro > [name].urdf

~Run Moveit Setup Assistant (https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html)
roslaunch moveit_setup_assistant setup_assistant.launch


############################
# Robotiq 3-Finger Gripper #
############################
https://github.com/TAMS-Group/robotiq

https://github.com/TAMS-Group/robotiq_s_model_action_server


######################################################################
wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.71-rt51.patch.xz
wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.71-rt51.patch.sign
wget https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.15.71.tar.xz
wget https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.15.71.tar.sign_

xz -dk patch-5.14.2-rt21.patch.xz
xz -d linux-5.14.2.tar.xz

gpg --keyserver keyserver.ubuntu.com --search williams

sudo sed -i 's/^GRUB_DEFAULT=.*/GRUB_DEFAULT="Advanced options for Ubuntu>Ubuntu, with Linux 5.15.71-rt51"/' /etc/default/grub


sudo sbsign --key MOK.priv --cert MOK.pem /boot/vmlinuz-5.15.71-rt51 --output /boot/vmlinuz-5.15.71-rt51.signed

sudo cp /boot/initrd.img-5.15.71-rt51{,.signed}

sudo mv /boot/vmlinuz-5.15.71-rt51{.signed,}
sudo mv /boot/initrd.img-5.15.71-rt51{.signed,}
sudo update-grub




https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git


Git Token
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





```
# we need to make sure you have all dependencies installed.

cd ~/catkin_ws
rosdep update

rosdep install --from-paths src/ --ignore-src --rosdistro noetic

# now build
catkin_make
```


```

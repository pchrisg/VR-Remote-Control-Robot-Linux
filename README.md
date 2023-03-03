##############################
# Universal Robot ROS DRIVER #
##############################
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
https://github.com/ros-industrial/universal_robot

~~~~~~~~~~~~
~Base Setup~
~~~~~~~~~~~~
# source global ros
source /opt/ros/noetic/setup.bash

# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the description. Currently, it is necessary to use the melodic-devel-staging branch.cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git src/universal_robot

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make

# activate the workspace (ie: source it)
source devel/setup.bash

~~~~~~~~~~~~~
~Calibration~
~~~~~~~~~~~~~
cd catkin_ws/src
catkin_create_pkg chris_ur_launch ur_client_library -D "Package containing calibrations and launch files for our UR robots."
mkdir -p chris_ur_launch/etc
mkdir -p chris_ur_launch/launch

roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.56.5 target_filename:="$(rospack find chris_ur_launch)/etc/ur5_calibration.yaml"

roscp ur_robot_driver ur5_bringup.launch ur5.launch

# Next, modify the parameter section of the new launchfile to match your actual calibration
# Note: Only the relevant lines are printed here
  <arg name="robot_ip" default="192.168.56.5"/>
  <arg name="kinematics_config" default="$(find chris_ur_launch)/etc/ur5_calibration.yaml"/>

~~~~~~~~~~~
~Execution~
~~~~~~~~~~~
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.56.5
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz

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
roslaunch ur5_moveit_config moveit_planning_execution.launch
roslaunch chris_ur5_moveit chris_ur5_moveit.launch

~~
roslaunch robotiq_gazebo robotiq.launch
rosrun robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_joint_states _model:=gazebo

rosrun robotiq_3f_gripper_control Robotiq3FGripperController.py gazebo


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
rosrun xacro xacro [name].xacro --inorder > [name].urdf

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
#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from chris_ur5_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
#if sys.version_info >= (3, 0):
#    def planCompat(plan):
#        return plan[1]
#else:
#    def planCompat(plan):
#        return plan
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return plan[1]


def go_to_destination(req):
    response = MoverServiceResponse()

    group_name = "ur5_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    response.trajectory = plan_trajectory(move_group, req.destination, req.joints_input.joints)

    move_group.execute(response.trajectory)
    move_group.clear_pose_targets()

    return response


def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('chris_ur5_moveit_server')

    s = rospy.Service('chris_ur5_moveit', MoverService, go_to_destination)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()

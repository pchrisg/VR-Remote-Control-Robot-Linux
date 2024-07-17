// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MoveIt Messages
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotState.h>

// Std Messages
// #include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"

#include <cmath>

// Name of the movegroup
static const std::string PLANNING_GROUP = "manipulator";

// We use the :planning_interface:`MoveGroupInterface` class to control the movegroup
moveit::planning_interface::MoveGroupInterface *m_Ur5;

// We use the :planning_interface:`PlanningSceneInterface` class to add and remove collision objects in our "virtual world" scene
moveit::planning_interface::PlanningSceneInterface *m_Scene;
std::vector<moveit_msgs::CollisionObject> *m_CollisionObjects;

std::vector<double> m_StartJointValues{0.0f, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2};

bool m_isLocked = false;
bool m_isSuccess = true;

void get_basic_info()
{
	// Getting Basic Information

	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("info", "Planning frame: %s", (*m_Ur5).getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("info", "End effector link: %s", (*m_Ur5).getEndEffectorLink().c_str());

	// We can get a list of all the groups in the robot:
	// ROS_INFO_NAMED("info", "Available Planning Groups:");
	// std::copy((*m_Ur5).getJointModelGroupNames().begin(),
	//          (*m_Ur5).getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

void reset_pose(const std_msgs::Empty::ConstPtr &msg)
{
	ros::AsyncSpinner startPosSpinner(1);
	startPosSpinner.start();

	ROS_INFO_NAMED("info", "Resetting Pose");

	(*m_Ur5).setJointValueTarget(m_StartJointValues);
	(*m_Ur5).move();

	startPosSpinner.stop();
}

/*void execute_plan(const moveit_msgs::RobotTrajectory::ConstPtr &traj)
{
	ros::AsyncSpinner excPlanSpinner(1);
	excPlanSpinner.start();

	(*m_Ur5).execute(*traj);
	excPlanSpinner.stop();
}*/

void move_arm(const geometry_msgs::Pose::ConstPtr &pose)
{
	if (!m_isLocked)
	{
		ros::AsyncSpinner moveArmSpinner(1);
		moveArmSpinner.start();

		moveit::core::RobotState start_state(*(m_Ur5->getCurrentState()));
		(*m_Ur5).setStartState(start_state);
		(*m_Ur5).setPoseTarget(*pose);

		moveit::planning_interface::MoveGroupInterface::Plan myPlan;
		m_isSuccess = ((*m_Ur5).plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (m_isSuccess)
		{
			std::vector<double> currjointvals = (*m_Ur5).getCurrentJointValues();

			int size = myPlan.trajectory_.joint_trajectory.points.size();
			std::vector<double> endJointValues = myPlan.trajectory_.joint_trajectory.points[size - 1].positions;

			for (int i = 0; i < currjointvals.size(); i = i + currjointvals.size() - 1)
			{
				if (std::fabs(currjointvals[i] - endJointValues[i]) > M_PI)
				{
					ROS_WARN("Fail: ABORTED: Bad motion plan found. No execution attempted.");
					m_isSuccess = false;
					break;
				}
			}
			if (m_isSuccess)
			{
				std::vector<double> midJointValues = myPlan.trajectory_.joint_trajectory.points[size / 2].positions;
				for (int i = currjointvals.size() - 1; i < currjointvals.size(); i = i + currjointvals.size() - 1)
				{
					if (std::fabs(midJointValues[i] - endJointValues[i]) > std::fabs(currjointvals[i] - endJointValues[i]))
					{
						// ROS_WARN("Fail: ABORTED: Suboptimal motion plan. No execution attempted.");
						m_isSuccess = false;
					}
				}
			}

			if (m_isSuccess)
				(*m_Ur5).execute(myPlan);
		}

		moveArmSpinner.stop();
	}
}

void emergency_stop(const std_msgs::Bool::ConstPtr &msg)
{
	if ((*msg).data)
	{
		ros::AsyncSpinner emergencyStopSpinner(1);
		emergencyStopSpinner.start();

		(*m_Ur5).stop();
		ROS_INFO("robot stopped due to collision");
		m_isLocked = true;

		emergencyStopSpinner.stop();
	}
	else
	{
		m_isLocked = false;
		ROS_INFO("unlocked");
	}
}

void remove_collision_objects_by_id(std::vector<std::string> objectIds)
{
	(*m_Scene).removeCollisionObjects(objectIds);

	for (int i = 0; i < objectIds.size(); i++)
		ROS_INFO("removed object %s from planning scene", objectIds[i].c_str());
}

void remove_collision_object(const moveit_msgs::CollisionObject::ConstPtr &collisionObject)
{
	// Now, let's remove the objects from the world.
	std::vector<std::string> objectIds;
	objectIds.push_back((*collisionObject).id);

	remove_collision_objects_by_id(objectIds);

	for (int i = 0; i < (*m_CollisionObjects).size(); i++)
	{
		if ((*collisionObject).id == (*m_CollisionObjects)[i].id)
			(*m_CollisionObjects).erase((*m_CollisionObjects).begin() + i);
	}
}

void remove_all_collision_objects()
{
	std::vector<std::string> objectIds;

	for (int i = 0; i < (*m_CollisionObjects).size(); i++)
		objectIds.push_back((*m_CollisionObjects)[i].id);

	remove_collision_objects_by_id(objectIds);

	(*m_CollisionObjects).clear();
	(*m_CollisionObjects).resize(0);
}

void add_collision_object(const moveit_msgs::CollisionObject::ConstPtr &collisionObject)
{
	if ((*collisionObject).id == "-1")
	{
		if ((*m_CollisionObjects).size() == 0)
			ROS_INFO("no old objects to remove");
		else
		{
			ROS_INFO("removing old objects");
			remove_all_collision_objects();
		}

		return;
	}

	(*m_CollisionObjects).push_back(*collisionObject);
	(*m_Scene).addCollisionObjects(*m_CollisionObjects);
	ROS_INFO("added object %s to planning scene", (*collisionObject).id.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "chris_ur5_moveit");
	ros::NodeHandle ur5nh("/ur5");

	moveit::planning_interface::MoveGroupInterface::Options options(PLANNING_GROUP);
	options.node_handle_ = ur5nh;

	m_Ur5 = new moveit::planning_interface::MoveGroupInterface(options);
	//(*m_Ur5).setPlanningTime(0.8);

	m_Scene = new moveit::planning_interface::PlanningSceneInterface("/ur5");
	m_CollisionObjects = new std::vector<moveit_msgs::CollisionObject>();

	ros::Publisher planSuccesPub = ur5nh.advertise<std_msgs::Bool>("chris_plan_success", 1);

	ros::Subscriber resetPoseSub = ur5nh.subscribe("/chris_reset_pose", 1, reset_pose);
	// ros::Subscriber executeSub = nodeHandle.subscribe("/chris_execute_plan", 1, execute_plan);
	ros::Subscriber moveArmSub = ur5nh.subscribe("/chris_move_arm", 1, move_arm);
	ros::Subscriber emgStpSub = ur5nh.subscribe("/chris_emergency_stop", 1, emergency_stop);
	ros::Subscriber addColObjSub = ur5nh.subscribe("/chris_add_collision_object", 1, add_collision_object);
	ros::Subscriber remColObjSub = ur5nh.subscribe("/chris_remove_collision_object", 1, remove_collision_object);

	get_basic_info();

	ros::AsyncSpinner mainSpinner(1);
	mainSpinner.start();
	ros::Rate rate(100);

	while (ros::ok())
	{
		std_msgs::Bool msg;
		msg.data = m_isSuccess;
		planSuccesPub.publish(msg);

		rate.sleep();
	}

	ros::waitForShutdown();

	return 0;
}
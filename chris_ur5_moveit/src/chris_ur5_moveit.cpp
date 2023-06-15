#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>

//#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <cmath>

#include <chris_ur5_moveit/TrajectoryPlannerService.h>
#include <chris_ur5_moveit/SdofTranslation.h>
#include <chris_ur5_moveit/SdofRotation.h>

// Name of the movegroup
static const std::string PLANNING_GROUP = "manipulator";

// We use the :planning_interface:`MoveGroupInterface` class to control the movegroup
moveit::planning_interface::MoveGroupInterface* m_Ur5;

// We use the :planning_interface:`PlanningSceneInterface` class to add and remove collision objects in our "virtual world" scene
moveit::planning_interface::PlanningSceneInterface* m_Scene;

std::vector<moveit_msgs::CollisionObject>* m_CollisionObjects;

std::vector<double> m_StartJointValues{0.0f, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, 0.0f};

void get_basic_info()
{
  // Getting Basic Information

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("info", "Planning frame: %s", (*m_Ur5).getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("info", "End effector link: %s", (*m_Ur5).getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  //ROS_INFO_NAMED("info", "Available Planning Groups:");
  //std::copy((*m_Ur5).getJointModelGroupNames().begin(),
  //          (*m_Ur5).getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

void reset_pose(const std_msgs::Empty::ConstPtr& msg)
{
  ros::AsyncSpinner startPosSpinner(1);
  startPosSpinner.start();

  ROS_INFO_NAMED("info", "Resetting Pose");

  (*m_Ur5).setJointValueTarget(m_StartJointValues);
  (*m_Ur5).move();

  startPosSpinner.stop();
}

bool plan_trajectory(chris_ur5_moveit::TrajectoryPlannerService::Request &req,
                     chris_ur5_moveit::TrajectoryPlannerService::Response &res)
{
  ros::AsyncSpinner planTrajSpinner(1);
  planTrajSpinner.start();

  (*m_Ur5).setPoseTarget(req.destination);
  moveit::planning_interface::MoveGroupInterface::Plan myPlan;

  bool success = ((*m_Ur5).plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success)
  {
    res.trajectory = myPlan.trajectory_;
    ROS_INFO_NAMED("info", "trajectory sent");
  }

  planTrajSpinner.stop();
  return true;
}

void execute_plan(const moveit_msgs::RobotTrajectory::ConstPtr& traj)
{
  ros::AsyncSpinner excPlanSpinner(1);
  excPlanSpinner.start();

  (*m_Ur5).execute(*traj);

  excPlanSpinner.stop();
}

void move_arm(const geometry_msgs::Pose::ConstPtr& pose)
{
  ros::AsyncSpinner moveArmSpinner(1);
  moveArmSpinner.start();

  (*m_Ur5).setPoseTarget(*pose);
  (*m_Ur5).move();

  moveArmSpinner.stop();
}

void emergency_stop(const std_msgs::Empty::ConstPtr& msg)
{
  (*m_Ur5).stop();
  ROS_INFO("robot stopped due to collision");
}

/*void sdof_translate(const chris_ur5_moveit::SdofTranslation::ConstPtr& tmsg)
{
  ros::AsyncSpinner sdofTSpinner(1);
  sdofTSpinner.start();

  moveit_msgs::Constraints rotCons;
  rotCons.orientation_constraints.push_back((*tmsg).orientation_constraint);
  (*m_Ur5).setPathConstraints(rotCons);
  (*m_Ur5).setPoseTarget((*tmsg).destination);
  (*m_Ur5).move();


  (*m_Ur5).clearPathConstraints();
  sdofTSpinner.stop();
}

void sdof_rotate(const chris_ur5_moveit::SdofRotation::ConstPtr& rmsg)
{
  ros::AsyncSpinner sdofRSpinner(1);
  sdofRSpinner.start();

  moveit_msgs::Constraints tranCons;
  tranCons.position_constraints.push_back(rmsg->position_constraint);
  m_Ur5->setPathConstraints(tranCons);
  m_Ur5->setPoseTarget(rmsg->destination);
  m_Ur5->move();


  m_Ur5->clearPathConstraints();
  sdofRSpinner.stop();
}*/

void remove_collision_objects_by_id(std::vector<std::string> objectIds)
{
  (*m_Scene).removeCollisionObjects(objectIds);

  for(int i = 0; i < objectIds.size(); i++)
  {
    ROS_INFO("removed collision object %s from planning scene", objectIds[i].c_str());
  }
}

void remove_collision_object(const moveit_msgs::CollisionObject::ConstPtr& collisionObject)
{
  // Now, let's remove the objects from the world.
  std::vector<std::string> objectIds;
  objectIds.push_back((*collisionObject).id);

  remove_collision_objects_by_id(objectIds);

  for(int i = 0; i < (*m_CollisionObjects).size(); i++)
  {
    if((*collisionObject).id == (*m_CollisionObjects)[i].id)
      (*m_CollisionObjects).erase((*m_CollisionObjects).begin() + i);
  }
}

void remove_all_collision_objects()
{
  std::vector<std::string> objectIds;
  
  for(int i = 0; i < (*m_CollisionObjects).size(); i++)
  {
    objectIds.push_back((*m_CollisionObjects)[i].id);
  }
  remove_collision_objects_by_id(objectIds);

  (*m_CollisionObjects).clear();
  (*m_CollisionObjects).resize(0);
}

void add_collision_object(const moveit_msgs::CollisionObject::ConstPtr& collisionObject)
{
  if((*collisionObject).id == "-1")
  {
    if ((*m_CollisionObjects).size() == 0)
      ROS_INFO("no old collision objects to remove");
    else
    {
      ROS_INFO("removing old collision objects");
      remove_all_collision_objects();
    }
    
    return;
  }

  /*for(int i = 0; i < (*m_CollisionObjects).size(); i++)
  {
    if((*collisionObject).id == (*m_CollisionObjects)[i].id)
      remove_collision_object(collisionObject);
  }*/

  (*m_CollisionObjects).push_back(*collisionObject);

  (*m_Scene).addCollisionObjects(*m_CollisionObjects);
    ROS_INFO("adding collision object %s to planning scene", (*collisionObject).id.c_str());
}

void attach_collision_object(const moveit_msgs::AttachedCollisionObject::ConstPtr& attachedCollisionObject)
{
  /*for(int i = 0; i < (*m_CollisionObjects).size(); i++)
  {
    if((*attachableObject).id == (*m_CollisionObjects)[i].id)
      remove_collision_object(attachableObject);
  }*/

  //(*m_CollisionObjects).push_back(*attachableObject);
  //(*m_Scene).applyCollisionObject(*attachableObject);

  std::map<std::string,moveit_msgs::CollisionObject> objects = (*m_Scene).getObjects();

  if(objects.size() == 0)
    ROS_INFO("no objects found in planning scene");
  else
  {
    ROS_INFO("objects in planning scene");
    for (auto const &pair: objects)
      ROS_INFO("%s", pair.first.c_str());
  }
  

  if((*m_Scene).applyAttachedCollisionObject(*attachedCollisionObject))
    ROS_INFO("applied attached collision object %s to planning scene", (*attachedCollisionObject).object.id.c_str());

  if((*m_Ur5).attachObject((*attachedCollisionObject).object.id, (*attachedCollisionObject).link_name))
    ROS_INFO("attached collision object %s to %s", (*attachedCollisionObject).object.id.c_str(), (*attachedCollisionObject).link_name.c_str());
}

void detach_collision_object(const moveit_msgs::CollisionObject::ConstPtr& collisionObject)
{
  if((*m_Ur5).detachObject((*collisionObject).id))
    ROS_INFO("detached collision object %s from UR5", (*collisionObject).id.c_str());
}

/*Attaching objects to the robot
{
  // 
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // You can attach objects to the robot, so that it moves with the robot geometry.
  // This simulates picking up the object for the purpose of manipulating it.
  // The motion planning should avoid collisions between the two objects as well.
  moveit_msgs::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  // We define the frame/pose for this cylinder so that it appears in the gripper
  object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
  geometry_msgs::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.z = 0.2;

  // First, we add the object to the world (without using a vector)
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  move_group_interface.attachObject(object_to_attach.id, "panda_hand");


  // Now, let's detach the cylinder from the robot's gripper.
  move_group_interface.detachObject(object_to_attach.id);
}
*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "chris_ur5_moveit");
  ros::NodeHandle nodeHandle;

  m_Ur5 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  m_Scene = new moveit::planning_interface::PlanningSceneInterface();
  m_CollisionObjects = new std::vector<moveit_msgs::CollisionObject>();
  (*m_Ur5).setPlanningTime(0.8);

  ros::ServiceServer plannerSrv = nodeHandle.advertiseService("chris_plan_trajectory", plan_trajectory);

  ros::Subscriber resetPoseSub = nodeHandle.subscribe("chris_reset_pose", 1, reset_pose);
  ros::Subscriber executeSub = nodeHandle.subscribe("chris_execute_plan", 1, execute_plan);
  ros::Subscriber moveArmSub = nodeHandle.subscribe("chris_move_arm", 5, move_arm);
  ros::Subscriber emgStpSub = nodeHandle.subscribe("chris_emergency_stop", 1, emergency_stop);
  ros::Subscriber addColObjSub = nodeHandle.subscribe("chris_add_collision_object", 1, add_collision_object);
  ros::Subscriber remColObjSub = nodeHandle.subscribe("chris_remove_collision_object", 1, remove_collision_object);
  ros::Subscriber attColObjSub = nodeHandle.subscribe("chris_attach_collision_object", 1, attach_collision_object);
  ros::Subscriber detColObjSub = nodeHandle.subscribe("chris_detach_collision_object", 1, detach_collision_object);

  //ros::Subscriber sdof_trans_sub = node_handle.subscribe("chris_sdof_translate", 1, sdof_translate);
  //ros::Subscriber sdof_rot_sub = node_handle.subscribe("chris_sdof_rotate", 1, sdof_rotate);
  //ros::Subscriber exit_game_sub = node_handle.subscribe("chris_game_exit", 1, game_exit);
  
  get_basic_info();
  //reset_pose();

  ros::AsyncSpinner mainSpinner(1);
  mainSpinner.start();
  ros::waitForShutdown();
  return 0;
}
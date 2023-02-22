#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
//#include "std_msgs/String.h"
//#include "std_msgs/Empty.h"

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

void get_basic_info(moveit::planning_interface::MoveGroupInterface move_group_interface)
{
  // Getting Basic Information

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("info", "Planning frame: %s", (*m_Ur5).getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("info", "End effector link: %s", (*m_Ur5).getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("info", "Available Planning Groups:");
  std::copy((*m_Ur5).getJointModelGroupNames().begin(),
            (*m_Ur5).getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

bool plan_trajectory(chris_ur5_moveit::TrajectoryPlannerService::Request &req,
                     chris_ur5_moveit::TrajectoryPlannerService::Response &res)
{
  ros::AsyncSpinner plan_trajectory_spinner(1);
  plan_trajectory_spinner.start();

  (*m_Ur5).setPoseTarget(req.destination);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = ((*m_Ur5).plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success)
  {
    res.trajectory = my_plan.trajectory_;
    ROS_INFO("trajectory sent");
  }

  plan_trajectory_spinner.stop();
  return true;
}

void execute_plan(const moveit_msgs::RobotTrajectory::ConstPtr& traj)
{
  ros::AsyncSpinner execute_plan_spinner(1);
  execute_plan_spinner.start();
  (*m_Ur5).execute(*traj);
  execute_plan_spinner.stop();
}

void move_arm(const geometry_msgs::Pose::ConstPtr& pose)
{
  ros::AsyncSpinner move_arm_spinner(1);
  move_arm_spinner.start();

  (*m_Ur5).setPoseTarget(*pose);
  (*m_Ur5).move();

  move_arm_spinner.stop();
}

void sdof_translate(const chris_ur5_moveit::SdofTranslation::ConstPtr& tmsg)
{
  ros::AsyncSpinner sdof_tspinner(1);
  sdof_tspinner.start();

  moveit_msgs::Constraints rotCons;
  rotCons.orientation_constraints.push_back((*tmsg).orientation_constraint);
  (*m_Ur5).setPathConstraints(rotCons);
  (*m_Ur5).setPoseTarget((*tmsg).destination);
  (*m_Ur5).move();


  (*m_Ur5).clearPathConstraints();
  sdof_tspinner.stop();
}

/*void sdof_rotate(const chris_ur5_moveit::SdofRotation::ConstPtr& rmsg)
{
  ros::AsyncSpinner sdof_rspinner(1);
  sdof_rspinner.start();

  moveit_msgs::Constraints tranCons;
  tranCons.position_constraints.push_back(rmsg->position_constraint);
  m_Ur5->setPathConstraints(tranCons);
  m_Ur5->setPoseTarget(rmsg->destination);
  m_Ur5->move();


  m_Ur5->clearPathConstraints();
  sdof_rspinner.stop();
}
*/

void remove_collision_objects_by_id(std::vector<std::string> object_ids)
{
  m_Scene->removeCollisionObjects(object_ids);

  for(int i = 0; i < object_ids.size(); i++)
  {
    ROS_INFO("removed collision object %s", object_ids[i].c_str());
  }
}

void remove_collision_object(const moveit_msgs::CollisionObject::ConstPtr& collisionObject)
{
  // Now, let's remove the objects from the world.
  std::vector<std::string> object_ids;
  object_ids.push_back((*collisionObject).id);
  remove_collision_objects_by_id(object_ids);

  for(int i = 0; i < (*m_CollisionObjects).size(); i++)
  {
    if(collisionObject->id == (*m_CollisionObjects)[i].id)
      (*m_CollisionObjects).erase((*m_CollisionObjects).begin() + i);
  }
}

void remove_all_collision_objects()
{
  std::vector<std::string> object_ids;
  
  for(int i = 0; i < (*m_CollisionObjects).size(); i++)
  {
    object_ids.push_back((*m_CollisionObjects)[i].id);
  }
  remove_collision_objects_by_id(object_ids);

  (*m_CollisionObjects).clear();
  (*m_CollisionObjects).resize(0);
}

void add_collision_object(const moveit_msgs::CollisionObject::ConstPtr& collisionObject)
{
  if(collisionObject->id == "0" && (*m_CollisionObjects).size() != 0)
  {
    remove_all_collision_objects();
  }

  (*m_CollisionObjects).push_back(*collisionObject);
  (*m_Scene).addCollisionObjects(*m_CollisionObjects);

  ROS_INFO("added collision object %s", collisionObject->id.c_str());
}

/*
void add_object_to_attach()
{
  // Attaching objects to the robot
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
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group_interface.attachObject(object_to_attach.id, "panda_hand");


  // Now, let's detach the cylinder from the robot's gripper.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group_interface.detachObject(object_to_attach.id);

  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to receive and process the attached collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");
}
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chris_ur5_moveit");
  ros::NodeHandle node_handle;

  m_Ur5 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  m_Scene = new moveit::planning_interface::PlanningSceneInterface();
  m_CollisionObjects = new std::vector<moveit_msgs::CollisionObject>();
  (*m_Ur5).setPlanningTime(0.8);
  
  ros::ServiceServer planner_srv = node_handle.advertiseService("chris_plan_trajectory", plan_trajectory);
  ros::Subscriber execute_sub = node_handle.subscribe("chris_execute_plan", 1, execute_plan);
  ros::Subscriber move_arm_sub = node_handle.subscribe("chris_move_arm", 5, move_arm);
  ros::Subscriber sdof_trans_sub = node_handle.subscribe("chris_sdof_translate", 1, sdof_translate);
  //ros::Subscriber sdof_rot_sub = node_handle.subscribe("chris_sdof_rotate", 1, sdof_rotate);
  ros::Subscriber add_col_Obj_sub = node_handle.subscribe("chris_add_collision_object", 1, add_collision_object);
  ros::Subscriber rem_col_Obj_sub = node_handle.subscribe("chris_remove_collision_object", 1, remove_collision_object);
  //ros::Subscriber exit_game_sub = node_handle.subscribe("chris_game_exit", 1, game_exit);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
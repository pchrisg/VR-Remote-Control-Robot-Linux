// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

// MoveIt Messages
#include <moveit_msgs/RobotState.h>

// Chris' Moveit Messages
#include <chris_ur5_moveit/TrajectoryPlannerService.h>

// Name of the movegroup
static const std::string PLANNING_GROUP = "manipulator";

// We use the :planning_interface:`MoveGroupInterface` class to control the movegroup
moveit::planning_interface::MoveGroupInterface *m_Rf;

bool plan_trajectory(chris_ur5_moveit::TrajectoryPlannerService::Request &req,
                     chris_ur5_moveit::TrajectoryPlannerService::Response &res)
{
    ros::AsyncSpinner planTrajSpinner(1);
    planTrajSpinner.start();

    moveit::core::RobotState start_state(*(m_Rf->getCurrentState()));
    start_state.setFromIK((*m_Rf).getRobotModel()->getJointModelGroup(PLANNING_GROUP), req.start);
    (*m_Rf).setStartState(start_state);

    (*m_Rf).setPoseTarget(req.destination);
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;

    bool success = ((*m_Rf).plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        std::vector<double> currjointvals = (*m_Rf).getCurrentJointValues();

        int size = myPlan.trajectory_.joint_trajectory.points.size();
        std::vector<double> endJointValues = myPlan.trajectory_.joint_trajectory.points[size - 1].positions;

        for (int i = 0; i < currjointvals.size(); i = i + currjointvals.size() - 1)
        {
            if (std::fabs(currjointvals[i] - endJointValues[i]) > M_PI)
            {
                success = false;
                break;
            }
        }

        if (success)
            res.trajectory = myPlan.trajectory_;
        else
        {
            moveit_msgs::RobotTrajectory emptyTrajectory;
            res.trajectory = emptyTrajectory;
        }
    }

    planTrajSpinner.stop();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chris_robot_feedback");
    ros::NodeHandle rfnh("/feedback");

    moveit::planning_interface::MoveGroupInterface::Options options(PLANNING_GROUP);
    options.node_handle_ = rfnh;
    m_Rf = new moveit::planning_interface::MoveGroupInterface(options);
    (*m_Rf).setPlanningTime(0.1);

    ros::ServiceServer plannerSrv = rfnh.advertiseService("/chris_plan_trajectory", plan_trajectory);

    ros::AsyncSpinner mainSpinner(1);
    mainSpinner.start();

    ros::waitForShutdown();

    return 0;
}
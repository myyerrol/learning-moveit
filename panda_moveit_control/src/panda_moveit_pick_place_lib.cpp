#include <panda_moveit_control/panda_moveit_pick_place_lib.h>

// Global variables.
moveit::planning_interface::MoveGroupInterface       g_move_group("panda_arm");
moveit::planning_interface::MoveGroupInterface::Plan g_plan;

MoveItPickPlaceLib::MoveItPickPlaceLib()
{

    hand_goal_.trajectory.joint_names.push_back("joint_finger_l");
    hand_goal_.trajectory.joint_names.push_back("joint_finger_r");

    hand_goal_.trajectory.points.resize(1);
    hand_goal_.trajectory.points[0].positions.resize(2);
    hand_goal_.trajectory.points[0].velocities.resize(2);
    hand_goal_.trajectory.points[0].accelerations.resize(2);

    for (int i = 0; i < 2; i++) {
        hand_goal_.trajectory.points[0].positions[i]     = 0.0;
        hand_goal_.trajectory.points[0].velocities[i]    = 0.0;
        hand_goal_.trajectory.points[0].accelerations[i] = 0.0;
    }

    hand_trajectory_client_ = boost::make_shared<HandTrajectoryClient>(
        "panda_hand_controller/follow_joint_trajectory", true);

    while (!hand_trajectory_client_->waitForServer(ros::Duration(5)) ) {
        ROS_INFO(
            "Waiting for the hand joint_trajectory_action server...");
    }

    joint_model_group_ = g_move_group.getCurrentState()->getJointModelGroup(
        "panda_arm");

    ROS_INFO("Start moveit pick and place object demo...");
}

MoveItPickPlaceLib::~MoveItPickPlaceLib()
{

}

bool MoveItPickPlaceLib::moveToTarget(void)
{
    return (g_move_group.move() ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool MoveItPickPlaceLib::planToPoseTarget(geometry_msgs::Pose pose)
{
    g_move_group.setPoseTarget(pose);

    return (g_move_group.plan(g_plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool MoveItPickPlaceLib::planToPoseTargetWithConstraints(
    geometry_msgs::Pose pose,
    moveit_msgs::Constraints constraints,
    double planning_time)
{
    g_move_group.setPathConstraints(constraints);
    g_move_group.setPoseTarget(pose);
    g_move_group.setPlanningTime(planning_time);

    bool result = (g_move_group.plan(g_plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);

    g_move_group.clearPathConstraints();

    return result;
}

bool MoveItPickPlaceLib::planToJointValueTarget(
    std::vector<double> joint_positions)
{
    g_move_group.setJointValueTarget(joint_positions);

    return (g_move_group.plan(g_plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool MoveItPickPlaceLib::planToCartesianTarget(
    std::vector<geometry_msgs::Pose> waypoints,
    double velocity_scaling_factor,
    double end_effector_step,
    double jump_threshold)
{
    g_move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);

    moveit_msgs::RobotTrajectory trajectory;

    double fraction = g_move_group.computeCartesianPath(waypoints,
                                                        end_effector_step,
                                                        jump_threshold,
                                                        trajectory);

    if (fraction == -1.0) {
        ROS_INFO("Error, please check function!");
        return false;
    }
    else {
        ROS_INFO("Success, Cartesian path %.2f%% acheived!", fraction * 100);
        return true;
    }
}

std::vector<double> MoveItPickPlaceLib::getJointGroupPositions(
    moveit::core::RobotStatePtr current_state)
{
    std::vector<double> joint_positions;
    current_state->copyJointGroupPositions(joint_model_group_,
                                           joint_positions);
    return joint_positions;
}

#include <panda_moveit_control/panda_moveit_pick_place_lib.h>

// MoveGroupInterface global variables.
moveit::planning_interface::MoveGroupInterface       g_move_group("panda_arm");
moveit::planning_interface::MoveGroupInterface::Plan g_plan;
// MoveItVisualTools global variables.
namespace                              g_rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools g_visual_tools("link_arm_base");

MoveItPickPlaceLib::MoveItPickPlaceLib(void)
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

    g_visual_tools.deleteAllMarkers();
    g_visual_tools.loadRemoteControl();

    pose_text_ = Eigen::Affine3d::Identity();
    pose_text_.translation().z() = 1.75;

    ROS_INFO("Start moveit pick and place object demo...");
}

MoveItPickPlaceLib::~MoveItPickPlaceLib()
{

}

void MoveItPickPlaceLib::visualizeCartesianTarget(
    std::string text,
    std::vector<geometry_msgs::Pose> waypoints,
    bool prompt)
{
    g_visual_tools.deleteAllMarkers();
    g_visual_tools.publishText(pose_text_, text, g_rvt::WHITE, g_rvt::XLARGE);
    g_visual_tools.publishPath(waypoints, g_rvt::LIME_GREEN, g_rvt::SMALL);

    for (std::size_t i = 0; i < waypoints.size(); i++) {
        g_visual_tools.publishAxisLabeled(waypoints[i],
                                          "pt" + std::to_string(i),
                                          g_rvt::SMALL);
    }

    g_visual_tools.trigger();

    if (prompt) {
        g_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizeJointValueTarget(std::string text,
                                                   bool prompt)
{
    g_visual_tools.deleteAllMarkers();
    g_visual_tools.publishText(pose_text_, text, g_rvt::WHITE, g_rvt::XLARGE);
    g_visual_tools.publishTrajectoryLine(g_plan.trajectory_,
                                         joint_model_group_);
    g_visual_tools.trigger();

    if (prompt) {
        g_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizePlan(std::string text, bool prompt)
{
    g_visual_tools.deleteAllMarkers();
    g_visual_tools.publishText(pose_text_, text, g_rvt::WHITE, g_rvt::XLARGE);
    g_visual_tools.publishTrajectoryLine(g_plan.trajectory_,
                                         joint_model_group_);
    g_visual_tools.trigger();

    if (prompt) {
        g_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizePoseTarget(geometry_msgs::Pose pose_target,
                                             std::string label_target,
                                             std::string text,
                                             bool prompt)
{
    g_visual_tools.deleteAllMarkers();
    g_visual_tools.publishAxisLabeled(pose_target, label_target);
    g_visual_tools.publishText(pose_text_, text, g_rvt::WHITE, g_rvt::XLARGE);
    g_visual_tools.publishTrajectoryLine(g_plan.trajectory_,
                                         joint_model_group_);
    g_visual_tools.trigger();

    if (prompt) {
        g_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizePoseTargetWithConstraints(
    geometry_msgs::Pose pose_start,
    geometry_msgs::Pose pose_target,
    std::string label_start,
    std::string label_target,
    std::string text,
    bool prompt)
{
    g_visual_tools.deleteAllMarkers();
    g_visual_tools.publishAxisLabeled(pose_start, label_start);
    g_visual_tools.publishAxisLabeled(pose_target, label_target);
    g_visual_tools.publishText(pose_text_, text, g_rvt::WHITE, g_rvt::XLARGE);
    g_visual_tools.publishTrajectoryLine(g_plan.trajectory_,
                                         joint_model_group_);
    g_visual_tools.trigger();

    if (prompt) {
        g_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizeText(std::string text, bool prompt)
{
    g_visual_tools.deleteAllMarkers();
    g_visual_tools.publishText(pose_text_, text, g_rvt::WHITE, g_rvt::XLARGE);
    g_visual_tools.trigger();

    if (prompt) {
        g_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

bool MoveItPickPlaceLib::moveToTarget(void)
{
    return (g_move_group.move() ==
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

bool MoveItPickPlaceLib::planToJointValueTarget(
    std::vector<double> joint_positions)
{
    g_move_group.setJointValueTarget(joint_positions);

    return (g_move_group.plan(g_plan) ==
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

std::vector<double> MoveItPickPlaceLib::getJointGroupPositions(
    moveit::core::RobotStatePtr current_state)
{
    std::vector<double> joint_positions;
    current_state->copyJointGroupPositions(joint_model_group_,
                                           joint_positions);
    return joint_positions;
}

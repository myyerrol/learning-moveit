/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, myyerrol
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: myyerrol

#include <panda_moveit_control/panda_moveit_pick_place_lib.h>

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

    namespace                              rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_arm_base");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    pose_text_ = Eigen::Affine3d::Identity();
    pose_text_.translation().z() = 1.75;

    ROS_INFO("Finishing initialization of variables.");
}

MoveItPickPlaceLib::~MoveItPickPlaceLib()
{
}

void MoveItPickPlaceLib::addCollisionObject(
    std::string object_id,
    shape_msgs::SolidPrimitive object_primitive,
    geometry_msgs::Pose object_pose)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    collision_object.primitives.push_back(object_primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO("Adding an collision object into the world.");

    planning_scene_interface_.addCollisionObjects(collision_objects);
}

void MoveItPickPlaceLib::attachCollisionObject(std::string object_id)
{
    ROS_INFO("Attaching an collision object into the robot.");

    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    move_group.attachObject(object_id);
}

void MoveItPickPlaceLib::detachCollisionObject(std::string object_id)
{
    ROS_INFO("Detaching an collision object from the robot.");

    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    move_group.attachObject(object_id);
}

void MoveItPickPlaceLib::removeCollisionObject(std::string object_id)
{
    ROS_INFO("Removing an collision object from the world.");

    std::vector<std::string> object_ids;
    object_ids.push_back(object_id);

    planning_scene_interface_.removeCollisionObjects(object_ids);
}

void MoveItPickPlaceLib::visualizeCartesianTarget(
    std::string text,
    std::vector<geometry_msgs::Pose> waypoints,
    bool prompt)
{
    namespace                              rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_arm_base");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    visual_tools.publishText(pose_text_, text, rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);

    for (std::size_t i = 0; i < waypoints.size(); i++) {
        visual_tools.publishAxisLabeled(waypoints[i],
                                          "pt" + std::to_string(i),
                                          rvt::SMALL);
    }

    visual_tools.trigger();

    if (prompt) {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizeJointValueTarget(std::string text,
                                                   bool prompt)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(
        "panda_arm");

    namespace                              rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_arm_base");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    visual_tools.publishText(pose_text_, text, rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_,
                                         joint_model_group_);
    visual_tools.trigger();

    if (prompt) {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizePlan(std::string text, bool prompt)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(
        "panda_arm");

    namespace                              rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_arm_base");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    visual_tools.publishText(pose_text_, text, rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_,
                                         joint_model_group_);
    visual_tools.trigger();

    if (prompt) {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
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
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(
        "panda_arm");

    namespace                              rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_arm_base");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    visual_tools.publishAxisLabeled(pose_target, label_target);
    visual_tools.publishText(pose_text_, text, rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_,
                                         joint_model_group_);
    visual_tools.trigger();

    if (prompt) {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
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
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(
        "panda_arm");

    namespace                              rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_arm_base");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    visual_tools.publishAxisLabeled(pose_start, label_start);
    visual_tools.publishAxisLabeled(pose_target, label_target);
    visual_tools.publishText(pose_text_, text, rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_,
                                         joint_model_group_);
    visual_tools.trigger();

    if (prompt) {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

void MoveItPickPlaceLib::visualizeText(std::string text, bool prompt)
{
    namespace                              rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_arm_base");

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    visual_tools.publishText(pose_text_, text, rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    if (prompt) {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo.");
    }
    else {
        return;
    }
}

bool MoveItPickPlaceLib::executeToTarget(void)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    return (move_group.execute(plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool MoveItPickPlaceLib::moveToTarget(void)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    return (move_group.move() ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool MoveItPickPlaceLib::planToCartesianTarget(
    std::vector<geometry_msgs::Pose> waypoints,
    double velocity_scaling_factor,
    double end_effector_step,
    double jump_threshold)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);

    moveit_msgs::RobotTrajectory trajectory;

    double fraction = move_group.computeCartesianPath(waypoints,
                                                        end_effector_step,
                                                        jump_threshold,
                                                        trajectory);

    if (fraction == -1.0) {
        ROS_ERROR("Running computeCartesianPath() function failed!");
        return false;
    }
    else {
        ROS_INFO("%.2f%% cartesian paths are acheived successfully!",
                 fraction * 100);
        return true;
    }
}

bool MoveItPickPlaceLib::planToJointValueTarget(
    std::vector<double> joint_positions)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group.setJointValueTarget(joint_positions);

    return (move_group.plan(plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool MoveItPickPlaceLib::planToPoseTarget(geometry_msgs::Pose pose,
                                          bool move)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveItErrorCode error_code;
    error_code = move_group.plan(plan);
    bool result;

    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Planning arm's trajectory successfully!");
        result = true;
    }
    else {
        ROS_ERROR("Planning arm's trajectory failed!");
        result = false;
    }

    if (move) {
        if (move_group.move() ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Moveing arm successfully!");
        }
        else {
            ROS_ERROR("Moving arm failed!");
        }
    }

    return result;
}

bool MoveItPickPlaceLib::planToPoseTargetWithConstraints(
    geometry_msgs::Pose pose,
    moveit_msgs::Constraints constraints,
    double planning_time)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group.setPathConstraints(constraints);
    move_group.setPoseTarget(pose);
    move_group.setPlanningTime(planning_time);

    bool result = (move_group.plan(plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.clearPathConstraints();

    return result;
}

bool MoveItPickPlaceLib::pickSimpleObjectPipeline(void)
{
    ROS_INFO("Starting pick simple object pipeline...");

    ros::Duration(2.0).sleep();

    ROS_INFO(
        "Starting to plan arm's trajectory according to target pose 1.");

    geometry_msgs::Pose target_pose_1;
    target_pose_1.orientation.w = 1.0;
    target_pose_1.position.x = 0.0;
    target_pose_1.position.y = 0.8;
    target_pose_1.position.z = 0.4;

    planToPoseTarget(target_pose_1, true);
}

bool MoveItPickPlaceLib::placeSimpleObjectPipeline(void)
{
}

std::vector<double> MoveItPickPlaceLib::getJointGroupPositions(
    moveit::core::RobotStatePtr current_state)
{
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(
        "panda_arm");

    std::vector<double> joint_positions;
    current_state->copyJointGroupPositions(joint_model_group_,
                                           joint_positions);
    return joint_positions;
}

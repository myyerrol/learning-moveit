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

#ifndef PANDA_MOVEIT_PICK_PLACE_LIB_H
#define PANDA_MOVEIT_PICK_PLACE_LIB_H

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/shared_ptr.hpp>

typedef
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    HandTrajectoryClient;

typedef moveit::planning_interface::PlanningSceneInterface
    PlanningSceneInterface;

class MoveItPickPlaceLib
{
public:
    MoveItPickPlaceLib(void);
    ~MoveItPickPlaceLib(void);
    void addCollisionObject(std::string object_id,
                            shape_msgs::SolidPrimitive object_primitive,
                            geometry_msgs::Pose object_pose);
    void attachCollisionObject(std::string object_id);
    void detachCollisionObject(std::string object_id);
    void removeCollisionObject(std::string object_id);
    void visualizeCartesianTarget(std::string text,
                                  std::vector<geometry_msgs::Pose> waypoints,
                                  bool prompt);
    void visualizeJointValueTarget(std::string text, bool prompt);
    void visualizePlan(std::string text, bool prompt);
    void visualizePoseTarget(geometry_msgs::Pose pose_target,
                             std::string label_target,
                             std::string text,
                             bool prompt);
    void visualizePoseTargetWithConstraints(geometry_msgs::Pose pose_start,
                                            geometry_msgs::Pose pose_target,
                                            std::string label_start,
                                            std::string label_target,
                                            std::string text,
                                            bool prompt);
    void visualizeText(std::string text, bool prompt);
    bool executeToTarget(void);
    bool moveToTarget(void);
    bool planToCartesianTarget(std::vector<geometry_msgs::Pose> waypoints,
                               double velocity_scaling_factor = 0.1,
                               double end_effector_step = 0.01,
                               double jump_threshold = 0.0);
    bool planToJointValueTarget(std::vector<double> joint_positions);
    bool planToPoseTarget(geometry_msgs::Pose pose, bool move);
    bool planToPoseTargetWithConstraints(geometry_msgs::Pose pose,
                                         moveit_msgs::Constraints constraints,
                                         double planning_time = 10.0);
    bool pickSimpleObjectPipeline(void);
    bool placeSimpleObjectPipeline(void);
    std::vector<double> getJointGroupPositions(
        moveit::core::RobotStatePtr curent_state);
private:
    control_msgs::FollowJointTrajectoryGoal hand_goal_;
    boost::shared_ptr<HandTrajectoryClient> hand_trajectory_client_;
    PlanningSceneInterface                  planning_scene_interface_;
    const robot_state::JointModelGroup     *joint_model_group_;
    Eigen::Affine3d                         pose_text_;
};

#endif // PANDA_MOVEIT_PICK_PLACE_LIB_H

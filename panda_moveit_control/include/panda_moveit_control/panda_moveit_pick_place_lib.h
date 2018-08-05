#ifndef PANDA_MOVEIT_PICK_PLACE_LIB_H
#define PANDA_MOVEIT_PICK_PLACE_LIB_H

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <boost/shared_ptr.hpp>

typedef
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    HandTrajectoryClient;

class MoveItPickPlaceLib
{
public:
    MoveItPickPlaceLib();
    ~MoveItPickPlaceLib();
    bool moveToTarget(void);
    bool planToPoseTarget(geometry_msgs::Pose pose);
    bool planToPoseTargetWithConstraints(geometry_msgs::Pose pose,
                                         moveit_msgs::Constraints constraints,
                                         double planning_time = 10.0);
    bool planToJointValueTarget(std::vector<double> joint_positions);
    bool planToCartesianTarget(std::vector<geometry_msgs::Pose> waypoints,
                               double velocity_scaling_factor = 0.1,
                               double end_effector_step = 0.01,
                               double jump_threshold = 0.0);
    bool pickObject();
    bool placeOjbect();
    std::vector<double> getJointGroupPositions(
        moveit::core::RobotStatePtr curent_state);
private:
    control_msgs::FollowJointTrajectoryGoal hand_goal_;
    boost::shared_ptr<HandTrajectoryClient> hand_trajectory_client_;
    const robot_state::JointModelGroup     *joint_model_group_;
};

#endif // PANDA_MOVEIT_PICK_PLACE_LIB_H


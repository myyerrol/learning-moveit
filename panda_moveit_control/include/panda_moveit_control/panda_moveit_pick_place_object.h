#ifndef PANDA_MOVEIT_PICK_PLACE_OBJECT_H
#define PANDA_MOVEIT_PICK_PLACE_OBJECT_H

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <boost/shared_ptr.hpp>

typedef
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    HandTrajectoryClient;

class MoveItPickPlaceObject
{
public:
    MoveItPickPlaceObject();
    ~MoveItPickPlaceObject();
    void moveToTarget(void);
    bool planToPoseTarget(geometry_msgs::Pose pose);
    bool planToJointValueTarget(std::vector<double> joint_positions);
    bool planTo
    bool pickObject();
    bool placeOjbect();
    std::vector<double> getJointGroupPositions(
        moveit::core::RobotStatePtr curent_state);
private:
    control_msgs::FollowJointTrajectoryGoal hand_goal_;
    boost::shared_ptr<HandTrajectoryClient> hand_trajectory_client_;
    const robot_state::JointModelGroup     *joint_model_group_;
};

#endif // PANDA_MOVEIT_PICK_PLACE_OJBECT_H


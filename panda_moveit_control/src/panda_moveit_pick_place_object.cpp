#include <panda_moveit_control/panda_moveit_pick_place_object.h>

// Global variables.
moveit::planning_interface::MoveGroupInterface       g_move_group("panda_arm");
moveit::planning_interface::MoveGroupInterface::Plan g_plan;

MoveItPickPlaceObject::MoveItPickPlaceObject()
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
        ROS_INFO_STREAM(
            "Waiting for the hand joint_trajectory_action server...");
    }

    joint_model_group_ = g_move_group.getCurrentState()->getJointModelGroup(
        "panda_arm");

    ROS_INFO("Start moveit pick and place object demo...");
}

MoveItPickPlaceObject::~MoveItPickPlaceObject()
{

}

void MoveItPickPlaceObject::moveToPoseTarget(void)
{
    g_move_group.move();
}

bool MoveItPickPlaceObject::planToPoseTarget(geometry_msgs::Pose pose)
{
    g_move_group.setPoseTarget(pose);

    return (g_move_group.plan(g_plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool MoveItPickPlaceObject::planToJointValueTarget(
    std::vector<double> joint_positions)
{
    g_move_group.setJointValueTarget(joint_positions);

    return (g_move_group.plan(g_plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

std::vector<double> MoveItPickPlaceObject::getJointGroupPositions(
    moveit::core::RobotStatePtr current_state)
{
    std::vector<double> joint_positions;
    current_state->copyJointGroupPositions(joint_model_group_,
                                           joint_positions);
    return joint_positions;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_moveit_pick_place_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();
}

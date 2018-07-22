#ifndef PANDA_TELEOP_TRAJECTORY_KEYBOARD
#define PANDA_TELEOP_TRAJECTORY_KEYBOARD

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <termio.h>
#include <signal.h>
#include <sys/poll.h>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#define KEYCODE_A     0x61
#define KEYCODE_D     0x64
#define KEYCODE_E     0x65
#define KEYCODE_Q     0x71
#define KEYCODE_S     0x73
#define KEYCODE_W     0x77
#define KEYCODE_X     0x78
#define KEYCODE_Z     0x7A

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_E_CAP 0x45
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_X_CAP 0x58
#define KEYCODE_Z_CAP 0x5A

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajectoryClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
    GripperCmdClient;

class ArmTeleopTrajectoryKeyboard
{
public:
    ArmTeleopTrajectoryKeyboard();
    ~ArmTeleopTrajectoryKeyboard();
    void spinTeleopArm();
    void stopTeleopArm();
private:
    ros::NodeHandle                         panda_nh_;
    std::map<std::string, int>              arm_index_;
    double                                  arm_pos_step_;
    std::vector<std::string>                arm_name_;
    control_msgs::FollowJointTrajectoryGoal arm_goal_;
    double                                  gripper_pos_step_;
    control_msgs::GripperCommandGoal        gripper_goal_;
    boost::shared_ptr<TrajectoryClient>     trajectory_client_;
    boost::shared_ptr<GripperCmdClient>     gripper_cmd_client_;
};

#endif // PANDA_TELEOP_TRAJECTORY_KEYBOARD

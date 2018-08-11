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

#ifndef PANDA_TELEOP_ARM_HAND_KEYBOARD
#define PANDA_TELEOP_ARM_HAND_KEYBOARD

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

typedef
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    ArmTrajectoryClient;
// typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
//     HandCommandClient;
typedef
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    HandTrajectoryClient;

class TeleopArmHandKeyboard
{
public:
    TeleopArmHandKeyboard();
    ~TeleopArmHandKeyboard();
    void spinTeleopArmHand();
    void stopTeleopArmHand();
private:
    ros::NodeHandle                         panda_nh_;
    control_msgs::FollowJointTrajectoryGoal arm_goal_;
    control_msgs::FollowJointTrajectoryGoal hand_goal_;
    // control_msgs::GripperCommandGoal        hand_goal_;
    double                                  arm_position_step_;
    double                                  hand_position_step_;
    std::vector<std::string>                arm_name_;
    std::map<std::string, int>              arm_index_;
    boost::shared_ptr<ArmTrajectoryClient>  arm_trajectory_client_;
    boost::shared_ptr<HandTrajectoryClient> hand_trajectory_client_;
    // boost::shared_ptr<HandCommandClient>    hand_command_client_;
};

#endif // PANDA_TELEOP_ARM_HAND_KEYBOARD

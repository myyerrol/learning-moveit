#include <panda_teleop/panda_teleop_arm_hand_keyboard.h>

int            g_kfd = 0;
struct termios g_cooked;
struct termios g_raw;

TeleopArmHandKeyboard::TeleopArmHandKeyboard()
{
    arm_name_.push_back("joint_arm_1");
    arm_name_.push_back("joint_arm_2");
    arm_name_.push_back("joint_arm_3");
    arm_name_.push_back("joint_arm_4");
    arm_name_.push_back("joint_arm_5");
    arm_name_.push_back("joint_arm_6");
    arm_name_.push_back("joint_arm_7");

    arm_goal_.trajectory.joint_names.push_back("joint_arm_1");
    arm_goal_.trajectory.joint_names.push_back("joint_arm_2");
    arm_goal_.trajectory.joint_names.push_back("joint_arm_3");
    arm_goal_.trajectory.joint_names.push_back("joint_arm_4");
    arm_goal_.trajectory.joint_names.push_back("joint_arm_5");
    arm_goal_.trajectory.joint_names.push_back("joint_arm_6");
    arm_goal_.trajectory.joint_names.push_back("joint_arm_7");

    arm_goal_.trajectory.points.resize(1);
    arm_goal_.trajectory.points[0].positions.resize(arm_name_.size());
    arm_goal_.trajectory.points[0].velocities.resize(arm_name_.size());
    arm_goal_.trajectory.points[0].accelerations.resize(arm_name_.size());

    for (int i = 0; i < arm_name_.size(); i++) {
        arm_index_[arm_name_[i]] = i;
        arm_goal_.trajectory.points[0].positions[i]     = 0.0;
        arm_goal_.trajectory.points[0].velocities[i]    = 0.0;
        arm_goal_.trajectory.points[0].accelerations[i] = 0.0;
    }

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

    // hand_goal_.command.position   = 0.0;
    // hand_goal_.command.max_effort = 0.0;

    ros::NodeHandle n_private("~");
    n_private.param("arm_position_step", arm_position_step_, 0.0174);
    n_private.param("hand_position_step", hand_position_step_, 0.001);

    arm_trajectory_client_ =  boost::make_shared<ArmTrajectoryClient>(
        "panda_arm_controller/follow_joint_trajectory", true);

    hand_trajectory_client_ = boost::make_shared<HandTrajectoryClient>(
        "panda_hand_controller/follow_joint_trajectory", true);

    // hand_command_client_ = boost::make_shared<HandCommandClient>(
    //     "hand_controller/gripper_cmd", true);

    while (!arm_trajectory_client_->waitForServer(ros::Duration(5)) ) {
        ROS_INFO_STREAM(
            "Waiting for the arm joint_trajectory_action server...");
    }

    while (!hand_trajectory_client_->waitForServer(ros::Duration(5)) ) {
        ROS_INFO_STREAM(
            "Waiting for the hand joint_trajectory_action server...");
    }

    // while (!hand_command_client_->waitForServer(ros::Duration(5))) {
    //     ROS_INFO_STREAM("Waiting for the grippper_command_action server...");
    // }
}

TeleopArmHandKeyboard::~TeleopArmHandKeyboard()
{
    panda_nh_.shutdown();
}

void TeleopArmHandKeyboard::spinTeleopArmHand()
{
    char   keyboard_cmd;
    double arm_position[6];
    double hand_position = 0;
    bool   flag = false;

    memset(arm_position, 0, sizeof(arm_position));

    tcgetattr(g_kfd, &g_cooked);
    memcpy(&g_raw, &g_cooked, sizeof(struct termios));
    g_raw.c_lflag &=~ (ICANON | ECHO);
    g_raw.c_cc[VEOL] = 1;
    g_raw.c_cc[VEOF] = 2;
    tcsetattr(g_kfd, TCSANOW, &g_raw);

    puts("-----------------------------------------");
    puts("    Teleop Trajectory Arm By Keyboard    ");
    puts("                Version 1                ");
    puts("-----------------------------------------");
    puts("Q                   W                   E");
    puts("A                   S                   D");
    puts("Z                   X                    ");
    puts("                                         ");
    puts("-----------------------------------------");
    puts("Q:Joint1-UP  W:Joint2-UP  E:Joint3-UP    ");
    puts("A:Joint4-UP  S:Joint5-UP  D:Joint6-UP    ");
    puts("Z:Joint7-UP  X:Hand-Open                 ");
    puts("-----------------------------------------");
    puts("Shift+Q:Joint1-DOWN  Shift+W:Joint2-DOWN ");
    puts("Shift+E:Joint3-DOWN  Shift+A:Joint4-DOWN ");
    puts("Shift+S:Joint5-DOWN  Shift+D:Joint6-DOWN ");
    puts("Shift+Z:Joint7-DOWN                      ");
    puts("Shift+X:Hand-Close"                       );
    puts("-----------------------------------------");
    puts("PRESS CTRL-C TO QUIT                     ");

    struct pollfd ufd;
    ufd.fd = g_kfd;
    ufd.events = POLLIN;

    while (true) {
        boost::this_thread::interruption_point();
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("Function poll():");
            return;
        }
        else if (num > 0) {
            if(read(g_kfd, &keyboard_cmd, 1) < 0) {
                perror("Function read():");
                return;
            }
        }
        else {
            if(flag == true) {
                continue;
            }
        }

        switch(keyboard_cmd) {
            case KEYCODE_Q: {
                arm_position[arm_index_["joint_arm_1"]] += arm_position_step_;
                if (arm_position[arm_index_["joint_arm_1"]] >= 2.90) {
                    arm_position[arm_index_["joint_arm_1"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_W: {
                arm_position[arm_index_["joint_arm_2"]] += arm_position_step_;
                if (arm_position[arm_index_["joint_arm_2"]] >= 1.76) {
                    arm_position[arm_index_["joint_arm_2"]] = 1.76;
                }
                flag = true;
                break;
            }
            case KEYCODE_E: {
                arm_position[arm_index_["joint_arm_3"]] += arm_position_step_;
                if (arm_position[arm_index_["joint_arm_3"]] >= 2.90) {
                    arm_position[arm_index_["joint_arm_3"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_A: {
                arm_position[arm_index_["joint_arm_4"]] += arm_position_step_;
                if (arm_position[arm_index_["joint_arm_4"]] >= 0.0) {
                    arm_position[arm_index_["joint_arm_4"]] = 0.0;
                }
                break;
            }
            case KEYCODE_S: {
                arm_position[arm_index_["joint_arm_5"]] += arm_position_step_;
                if (arm_position[arm_index_["joint_arm_5"]] >= 2.90) {
                    arm_position[arm_index_["joint_arm_5"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_D: {
                arm_position[arm_index_["joint_arm_6"]] += arm_position_step_;
                if (arm_position[arm_index_["joint_arm_6"]] >= 3.75) {
                    arm_position[arm_index_["joint_arm_6"]] = 3.75;
                }
                flag = true;
                break;
            }
            case KEYCODE_Z: {
                arm_position[arm_index_["joint_arm_7"]] += arm_position_step_;
                if (arm_position[arm_index_["joint_arm_7"]] >= 2.90) {
                    arm_position[arm_index_["joint_arm_7"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_X: {
                hand_position += hand_position_step_;
                if (hand_position >= 0.04) {
                    hand_position = 0.04;
                }
                hand_goal_.trajectory.points[0].positions[0] = hand_position;
                hand_goal_.trajectory.points[0].positions[1] = hand_position;
                hand_goal_.trajectory.points[0].time_from_start =
                    ros::Duration(5);
                hand_goal_.goal_time_tolerance = ros::Duration(0);
                hand_trajectory_client_->sendGoal(hand_goal_);
                // hand_goal_.command.position = hand_position;
                // hand_command_client_->sendGoal(hand_goal_);
                flag = true;
                break;
            }
            case KEYCODE_Q_CAP: {
                arm_position[arm_index_["joint_arm_1"]] -= arm_position_step_;
                if (arm_position[arm_index_["joint_arm_1"]] <= -2.90) {
                    arm_position[arm_index_["joint_arm_1"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_W_CAP: {
                arm_position[arm_index_["joint_arm_2"]] -= arm_position_step_;
                if (arm_position[arm_index_["joint_arm_2"]] <= -1.76) {
                    arm_position[arm_index_["joint_arm_2"]] = -1.76;
                }
                flag = true;
                break;
            }
            case KEYCODE_E_CAP: {
                arm_position[arm_index_["joint_arm_3"]] -= arm_position_step_;
                if (arm_position[arm_index_["joint_arm_3"]] <= -2.90) {
                    arm_position[arm_index_["joint_arm_3"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_A_CAP: {
                arm_position[arm_index_["joint_arm_4"]] -= arm_position_step_;
                if (arm_position[arm_index_["joint_arm_4"]] <= -3.07) {
                    arm_position[arm_index_["joint_arm_4"]] = -3.07;
                }
                flag = true;
                break;
            }
            case KEYCODE_S_CAP: {
                arm_position[arm_index_["joint_arm_5"]] -= arm_position_step_;
                if (arm_position[arm_index_["joint_arm_5"]] <= -2.90) {
                    arm_position[arm_index_["joint_arm_5"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_D_CAP: {
                arm_position[arm_index_["joint_arm_6"]] -= arm_position_step_;
                if (arm_position[arm_index_["joint_arm_6"]] <= 0.0) {
                    arm_position[arm_index_["joint_arm_6"]] = 0.0;
                }
                break;
            }
            case KEYCODE_Z_CAP: {
                arm_position[arm_index_["joint_arm_7"]] -= arm_position_step_;
                if (arm_position[arm_index_["joint_arm_7"]] <= -2.90) {
                    arm_position[arm_index_["joint_arm_7"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_X_CAP: {
                hand_position -= hand_position_step_;
                if (hand_position <= 0.0) {
                    hand_position = 0.0;
                }
                hand_goal_.trajectory.points[0].positions[0] = hand_position;
                hand_goal_.trajectory.points[0].positions[1] = hand_position;
                hand_goal_.trajectory.points[0].time_from_start =
                    ros::Duration(5);
                hand_goal_.goal_time_tolerance = ros::Duration(0);
                hand_trajectory_client_->sendGoal(hand_goal_);
                // hand_goal_.command.position = hand_position;
                // hand_command_client_->sendGoal(hand_goal_);
                flag = true;
                break;
            }
            default: {
                flag = false;
            }
        }

        arm_goal_.trajectory.points[0].positions[arm_index_["joint_arm_1"]] =
            arm_position[arm_index_["joint_arm_1"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["joint_arm_2"]] =
            arm_position[arm_index_["joint_arm_2"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["joint_arm_3"]] = arm_position[arm_index_["joint_arm_3"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["joint_arm_4"]] = arm_position[arm_index_["joint_arm_4"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["joint_arm_5"]] =
            arm_position[arm_index_["joint_arm_5"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["joint_arm_6"]] = arm_position[arm_index_["joint_arm_6"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["joint_arm_7"]] = arm_position[arm_index_["joint_arm_7"]];

        arm_goal_.trajectory.points[0].time_from_start = ros::Duration(5);
        arm_goal_.goal_time_tolerance = ros::Duration(0);
        arm_trajectory_client_->sendGoal(arm_goal_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_teleop_arm_hand_keyboard",
              ros::init_options::NoSigintHandler);

    TeleopArmHandKeyboard teleop_arm_hand_keyboard;

    boost::thread make_thread = boost::thread(
        boost::bind(&TeleopArmHandKeyboard::spinTeleopArmHand,
                    &teleop_arm_hand_keyboard));

    ros::spin();

    make_thread.interrupt();
    make_thread.join();
    tcsetattr(g_kfd, TCSANOW, &g_cooked);

    return 0;
}

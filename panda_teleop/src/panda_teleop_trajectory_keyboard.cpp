#include <panda_teleop/panda_teleop_trajectory_keyboard.h>

int            g_kfd = 0;
struct termios g_cooked;
struct termios g_raw;

ArmTeleopTrajectoryKeyboard::ArmTeleopTrajectoryKeyboard()
{
    arm_name_.push_back("panda_joint1");
    arm_name_.push_back("panda_joint2");
    arm_name_.push_back("panda_joint3");
    arm_name_.push_back("panda_joint4");
    arm_name_.push_back("panda_joint5");
    arm_name_.push_back("panda_joint6");
    arm_name_.push_back("panda_joint7");

    arm_goal_.trajectory.joint_names.push_back("panda_joint1");
    arm_goal_.trajectory.joint_names.push_back("panda_joint2");
    arm_goal_.trajectory.joint_names.push_back("panda_joint3");
    arm_goal_.trajectory.joint_names.push_back("panda_joint4");
    arm_goal_.trajectory.joint_names.push_back("panda_joint5");
    arm_goal_.trajectory.joint_names.push_back("panda_joint6");
    arm_goal_.trajectory.joint_names.push_back("panda_joint7");

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

    ros::NodeHandle n_private("~");
    n_private.param("arm_pose_step", arm_pos_step_, 0.0174);
    n_private.param("gripper_pos_step", gripper_pos_step_, 0.01);

    gripper_pos_pub_ = panda_nh_.advertise<std_msgs::Float64>(
        "hand_controller/gripper_action/command", 1000);

    trajectory_client_ =  boost::make_shared<TrajectoryClient>(
        "panda_arm_controller/follow_joint_trajectory", true);

    while (!trajectory_client_->waitForServer(ros::Duration(5))) {
        ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
    }
}

ArmTeleopTrajectoryKeyboard::~ArmTeleopTrajectoryKeyboard()
{
    panda_nh_.shutdown();
}

void ArmTeleopTrajectoryKeyboard::spinTeleopArm()
{
    char   keyboard_cmd;
    double arm_position[6];
    double gripper_position = 0;
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
    puts("Z:Joint7-UP  X:Gripper Open              ");
    puts("-----------------------------------------");
    puts("Shift+Q:Joint1-DOWN  Shift+W:Joint2-DOWN ");
    puts("Shift+E:Joint3-DOWN  Shift+A:Joint4-DOWN ");
    puts("Shift+S:Joint5-DOWN  Shift+D:Joint6-DOWN ");
    puts("Shift+Z:Joint7-DOWN                      ");
    puts("Shift+X:Gripper Close"                    );
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
                arm_position[arm_index_["panda_joint1"]] += arm_pos_step_;
                if (arm_position[arm_index_["panda_joint1"]] >= 2.90) {
                    arm_position[arm_index_["panda_joint1"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_W: {
                arm_position[arm_index_["panda_joint2"]] += arm_pos_step_;
                if (arm_position[arm_index_["panda_joint2"]] >= 1.76) {
                    arm_position[arm_index_["panda_joint2"]] = 1.76;
                }
                flag = true;
                break;
            }
            case KEYCODE_E: {
                arm_position[arm_index_["panda_joint3"]] += arm_pos_step_;
                if (arm_position[arm_index_["panda_joint3"]] >= 2.90) {
                    arm_position[arm_index_["panda_joint3"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_A: {
                arm_position[arm_index_["panda_joint4"]] += arm_pos_step_;
                if (arm_position[arm_index_["panda_joint4"]] >= 0.0) {
                    arm_position[arm_index_["panda_joint4"]] = 0.0;
                }
                break;
            }
            case KEYCODE_S: {
                arm_position[arm_index_["panda_joint5"]] += arm_pos_step_;
                if (arm_position[arm_index_["panda_joint5"]] >= 2.90) {
                    arm_position[arm_index_["panda_joint5"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_D: {
                arm_position[arm_index_["panda_joint6"]] += arm_pos_step_;
                if (arm_position[arm_index_["panda_joint6"]] >= 3.75) {
                    arm_position[arm_index_["panda_joint6"]] = 3.75;
                }
                flag = true;
                break;
            }
            case KEYCODE_Z: {
                arm_position[arm_index_["panda_joint7"]] += arm_pos_step_;
                if (arm_position[arm_index_["panda_joint7"]] >= 2.90) {
                    arm_position[arm_index_["panda_joint7"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_X: {
                gripper_position += gripper_pos_step_;
                if (gripper_position >= 0.04) {
                    gripper_position = 0.04;
                }
                gripper_pos_.data = gripper_position;
                gripper_pos_pub_.publish(gripper_pos_);
                flag = true;
                break;
            }
            case KEYCODE_Q_CAP: {
                arm_position[arm_index_["panda_joint1"]] -= arm_pos_step_;
                if (arm_position[arm_index_["panda_joint1"]] <= -2.90) {
                    arm_position[arm_index_["panda_joint1"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_W_CAP: {
                arm_position[arm_index_["panda_joint2"]] -= arm_pos_step_;
                if (arm_position[arm_index_["panda_joint2"]] <= -1.76) {
                    arm_position[arm_index_["panda_joint2"]] = -1.76;
                }
                flag = true;
                break;
            }
            case KEYCODE_E_CAP: {
                arm_position[arm_index_["panda_joint3"]] -= arm_pos_step_;
                if (arm_position[arm_index_["panda_joint3"]] <= -2.90) {
                    arm_position[arm_index_["panda_joint3"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_A_CAP: {
                arm_position[arm_index_["panda_joint4"]] -= arm_pos_step_;
                if (arm_position[arm_index_["panda_joint4"]] <= -3.07) {
                    arm_position[arm_index_["panda_joint4"]] = -3.07;
                }
                flag = true;
                break;
            }
            case KEYCODE_S_CAP: {
                arm_position[arm_index_["panda_joint5"]] -= arm_pos_step_;
                if (arm_position[arm_index_["panda_joint5"]] <= -2.90) {
                    arm_position[arm_index_["panda_joint5"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_D_CAP: {
                arm_position[arm_index_["panda_joint6"]] -= arm_pos_step_;
                if (arm_position[arm_index_["panda_joint6"]] <= 0.0) {
                    arm_position[arm_index_["panda_joint6"]] = 0.0;
                }
                break;
            }
            case KEYCODE_Z_CAP: {
                arm_position[arm_index_["panda_joint7"]] -= arm_pos_step_;
                if (arm_position[arm_index_["panda_joint7"]] <= -2.90) {
                    arm_position[arm_index_["panda_joint7"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_X_CAP: {
                gripper_position -= gripper_pos_step_;
                if (gripper_position <= 0.0)
                    gripper_position = 0.0;
                gripper_pos_.data = gripper_position;
                gripper_pos_pub_.publish(gripper_pos_);
                flag = true;
                break;
            }
            default: {
                flag = false;
            }
        }

        arm_goal_.trajectory.points[0].positions[arm_index_["panda_joint1"]] =
            arm_position[arm_index_["panda_joint1"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["panda_joint2"]] =
            arm_position[arm_index_["panda_joint2"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["panda_joint3"]] = arm_position[arm_index_["panda_joint3"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["panda_joint4"]] = arm_position[arm_index_["panda_joint4"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["panda_joint5"]] =
            arm_position[arm_index_["panda_joint5"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["panda_joint6"]] = arm_position[arm_index_["panda_joint6"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["panda_joint7"]] = arm_position[arm_index_["panda_joint7"]];

        arm_goal_.trajectory.points[0].time_from_start = ros::Duration(5);
        arm_goal_.goal_time_tolerance = ros::Duration(0);
        trajectory_client_->sendGoal(arm_goal_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"panda_teleop_trajectory_keyboard",
        ros::init_options::NoSigintHandler);

    ArmTeleopTrajectoryKeyboard teleop_trajectory_keyboard;

    boost::thread make_thread = boost::thread(boost::bind(
        &ArmTeleopTrajectoryKeyboard::spinTeleopArm,
        &teleop_trajectory_keyboard));

    ros::spin();

    make_thread.interrupt();
    make_thread.join();
    tcsetattr(g_kfd, TCSANOW, &g_cooked);

    return 0;
}

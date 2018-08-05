#include <panda_moveit_control/panda_moveit_pick_place_lib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_moveit_pick_place_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();
}
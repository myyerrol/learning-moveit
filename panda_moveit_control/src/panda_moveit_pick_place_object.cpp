#include <panda_moveit_control/panda_moveit_pick_place_lib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_moveit_pick_place_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveItPickPlaceLib pick_place_object;

    pick_place_object.pickSimpleObjectPipeline();

    ros::shutdown();
    return 0;
}
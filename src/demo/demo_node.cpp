#include "demo.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "demo_node");

    rosparam_handler_tutorial::Demo demo(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}

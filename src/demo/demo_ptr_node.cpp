#include "demo_ptr.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "demo_ptr_node");

    // We give custom private sub-namespaces so
    // that rosparam_handler objects
    // do not collide.
    // More specifically the dynamic
    // reconfigure configCallback
    ros::NodeHandle foo_nh("~/foo");
    ros::NodeHandle bar_nh("~/bar");

    rosparam_handler_tutorial::Foo demo_foo(foo_nh);
    rosparam_handler_tutorial::Bar demo_bar(bar_nh);

    ros::spin();
    return 0;
}

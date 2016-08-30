#include "demo.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rosparam_handler_tutorial {

class DemoNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<Demo> m_;
};

void DemoNodelet::onInit() {
    m_.reset(new Demo(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace rosparam_handler_tutorial

PLUGINLIB_DECLARE_CLASS(rosparam_handler_tutorial, DemoNodelet, rosparam_handler_tutorial::DemoNodelet, nodelet::Nodelet);

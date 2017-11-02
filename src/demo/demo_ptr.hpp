#ifndef _ROSPARAM_HANDLER_TUTORIAL_DEMO_BASE_H_
#define _ROSPARAM_HANDLER_TUTORIAL_DEMO_BASE_H_

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "rosparam_handler_tutorial/FooParameters.h"
#include "rosparam_handler_tutorial/BarParameters.h"

namespace rosparam_handler_tutorial {

struct DemoBase
{
  DemoBase() = default;
  virtual ~DemoBase() = default;

  /// \brief A helper function to instantiate the
  /// params_ptr_ properly
  template <typename T>
  void init(ros::NodeHandle& private_node_handle);

  /// \brief A herlper function to call
  /// params_ptr_->fromParamServer
  void fromParamServer();

  /// \brief A herlper function to call
  /// params_ptr_->toParamServer
  void toParamServer();

  /// \brief A function to periodically print the params_ptr_
  void timerCallback(const ros::TimerEvent&) const;

  /// \brief A base pointer to the xParameters object
  rosparam_handler::ParametersPtr params_ptr_;

  ros::NodeHandle private_node_handle_;

  ros::Timer timer_;
};

template <typename T>
inline void DemoBase::init(ros::NodeHandle& private_node_handle)
{
  params_ptr_ = boost::make_shared<T>(private_node_handle);
}

struct Foo : public DemoBase
{
  using DemoBase::timerCallback;

  Foo(ros::NodeHandle private_node_handle);

  /// \brief The dynamic reconfigure callback
  void configCallback(FooConfig &config, uint32_t /*level*/);

  /// \brief The dynamic reconfigure server
  dynamic_reconfigure::Server<FooConfig> dr_srv_;
};

struct Bar : public DemoBase
{
  Bar(ros::NodeHandle private_node_handle);

  /// \brief The dynamic reconfigure callback
  void configCallback(BarConfig &config, uint32_t /*level*/);

  /// \brief The dynamic reconfigure server
  dynamic_reconfigure::Server<BarConfig> dr_srv_;
};

} // namespace rosparam_handler_tutorial

#endif /* _ROSPARAM_HANDLER_TUTORIAL_DEMO_BASE_H_ */

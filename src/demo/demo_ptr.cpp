#include "demo_ptr.hpp"

namespace rosparam_handler_tutorial {

void DemoBase::timerCallback(const ros::TimerEvent&) const {
  if (params_ptr_ != nullptr)
    ROS_INFO_STREAM("Parameter :\n" << *params_ptr_);
  else
    ROS_WARN("Timer callback executed before object params_ptr_ is instantiated !");
}

void DemoBase::fromParamServer()
{
  params_ptr_->fromParamServer();
}

void DemoBase::toParamServer()
{
  params_ptr_->toParamServer();
}

Foo::Foo(ros::NodeHandle private_node_handle) :
  DemoBase(), dr_srv_(private_node_handle)
{
  /**
   * Instantiation
   */
  init<FooParameters>(private_node_handle);

  /**
   * Initialization
   */
  fromParamServer();

  /**
   * Set up timer
   */

  /// @note We are certain that the object hold in params_ptr_
  /// is of type FooParameters and we don't want to pay for
  /// a dynamic_cast.
  FooParameters& param = *boost::static_pointer_cast<FooParameters>(params_ptr_);

  ros::TimerOptions timer_opt;
  timer_opt.oneshot   = false;
  timer_opt.autostart = true;

  timer_opt.callback = boost::bind(&Foo::timerCallback, this, _1);
  timer_opt.period   = ros::Rate(param.rate).expectedCycleTime();

  timer_ = private_node_handle.createTimer(timer_opt);

  /**
   * Set up dynamic reconfigure server
   */
  dynamic_reconfigure::Server<FooConfig>::CallbackType cb;
  cb = boost::bind(&Foo::configCallback, this, _1, _2);
  //dr_srv_.setCallback(cb);
}

void Foo::configCallback(FooConfig &config, uint32_t /*level*/)
{
  if (params_ptr_ != nullptr)
  {
    params_ptr_->fromConfig(config);

    int rate = boost::static_pointer_cast<FooParameters>(params_ptr_)->rate;

    timer_.setPeriod(ros::Rate(rate).expectedCycleTime());

    ROS_WARN_STREAM("Parameter update:\n" << *params_ptr_);
  }
  else
  {
    ROS_WARN("Dynamic reconfigure callback executed "
             "before parameters initialization !");
  }
}


Bar::Bar(ros::NodeHandle private_node_handle) :
  DemoBase(), dr_srv_(private_node_handle)
{
  /**
   * Instantiation
   */
  init<BarParameters>(private_node_handle);

  /**
   * Initialization
   */
  fromParamServer();

  /**
   * Set up timer
   */

  /// @note We are quite sure that the object hold in params_ptr_
  /// is of type BarParameters but we want to play it safe.

  boost::shared_ptr<BarParameters> bar_param_ptr =
      boost::dynamic_pointer_cast<BarParameters>(params_ptr_);

  int rate = 2;

  if (bar_param_ptr != nullptr)
  {
    rate = bar_param_ptr->rate;
  }
  else
  {
    ROS_WARN("Could not cast param_ptr_ to type BarParameters !"
             "\nUsing default rate (%i Hz)", rate);
  }

  ros::TimerOptions timer_opt;
  timer_opt.oneshot   = false;
  timer_opt.autostart = true;

  timer_opt.callback = boost::bind(&Bar::timerCallback, this, _1);
  timer_opt.period   = ros::Rate(rate).expectedCycleTime();

  timer_ = private_node_handle.createTimer(timer_opt);

  /**
     * Set up dynamic reconfigure server
     */
  dynamic_reconfigure::Server<BarConfig>::CallbackType cb;
  cb = boost::bind(&Bar::configCallback, this, _1, _2);
  //dr_srv_.setCallback(cb);
}

void Bar::configCallback(BarConfig &config, uint32_t /*level*/)
{
  if (params_ptr_ != nullptr)
  {
    params_ptr_->fromConfig(config);

    int rate = 2;

    boost::shared_ptr<BarParameters> bar_param_ptr =
        boost::dynamic_pointer_cast<BarParameters>(params_ptr_);

    if (bar_param_ptr != nullptr)
    {
      rate = bar_param_ptr->rate;
    }
    else
    {
      ROS_WARN("Could not cast param_ptr_ to type BarParameters !"
               "\nUsing default rate (%i Hz)", rate);
    }

    timer_.setPeriod(ros::Rate(rate).expectedCycleTime());

    ROS_WARN_STREAM("Parameter update:\n" << *params_ptr_);
  }
  else
  {
    ROS_WARN("Dynamic reconfigure callback executed "
             "before parameters initialization !");
  }
}

} // namespace rosparam_handler_tutorial

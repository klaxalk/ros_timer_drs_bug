#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <timer_tester/timer_testerConfig.h>

#define TIMERS_BEFORE 1

namespace timer_tester
{

class TimerTester : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef timer_tester::timer_testerConfig         DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(timer_tester::timer_testerConfig& config, uint32_t level);

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_fast_;
  void       timerFast(const ros::TimerEvent& te);

  ros::Timer timer_slow_;
  void       timerSlow(const ros::TimerEvent& te);
};

void TimerTester::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[TimerTester]: initializing");

#if TIMERS_BEFORE == 1
  ROS_INFO("[TimerTester]: creating timers before DRS");
  timer_fast_ = nh_.createTimer(ros::Rate(100.0), &TimerTester::timerFast, this);
  timer_slow_ = nh_.createTimer(ros::Rate(1.0), &TimerTester::timerSlow, this);
#endif

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  Drs_t::CallbackType f = boost::bind(&TimerTester::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

#if TIMERS_BEFORE == 0
  ROS_INFO("[TimerTester]: creating timers after DRS");
  timer_fast_ = nh_.createTimer(ros::Rate(100.0), &TimerTester::timerFast, this);
  timer_slow_ = nh_.createTimer(ros::Rate(1.0), &TimerTester::timerSlow, this);
#endif

  ROS_INFO_ONCE("[TimerTester]: initialized");
}

// | --------------------- timer callbacks -------------------- |

void TimerTester::timerFast([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO_THROTTLE(0.1, "[TimerTester]: 100 Hz timer spinning");
}

void TimerTester::timerSlow([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO_THROTTLE(0.1, "[TimerTester]: 1 Hz timer spinning");
}

void TimerTester::callbackDrs([[maybe_unused]] timer_tester::timer_testerConfig& config, [[maybe_unused]] uint32_t level) {

  ROS_INFO("[TimerTester]: callbackDrs() called");
}

}  // namespace timer_tester

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(timer_tester::TimerTester, nodelet::Nodelet);

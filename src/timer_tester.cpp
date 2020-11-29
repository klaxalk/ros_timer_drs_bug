#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <timer_tester/timer_testerConfig.h>

#include <std_msgs/Empty.h>

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

  ros::Subscriber topic_subscriber_;
  void            topicSubscriberCallback(const std_msgs::EmptyConstPtr& msg);
};

void TimerTester::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[TimerTester]: initializing");

#if TIMERS_BEFORE == 1
  ROS_INFO("[TimerTester]: creating timers before DRS");
  topic_subscriber_ = nh_.subscribe("in", 1, &TimerTester::topicSubscriberCallback, this, ros::TransportHints().tcpNoDelay());
  timer_fast_       = nh_.createTimer(ros::Rate(100.0), &TimerTester::timerFast, this);
  timer_slow_       = nh_.createTimer(ros::Rate(1.0), &TimerTester::timerSlow, this);
#endif

  // | --------------- dynamic reconfigure server --------------- |

  // initialize the DRS
  drs_.reset(new Drs_t(mutex_drs_, nh_));
  Drs_t::CallbackType f = boost::bind(&TimerTester::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // some empty loops that simulate more operations between the timer/subscriber initialization and the DRS
  // this could easily be some preparation of control matrices, or some library initialization
  for (long i = 0; i < 10e6; i++) {
    ROS_INFO_THROTTLE(0.1, "[TimerTester]: doing some small computation"); 
  }

#if TIMERS_BEFORE == 0
  ROS_INFO("[TimerTester]: creating timers after DRS");
  topic_subscriber_ = nh_.subscribe("in", 1, &TimerTester::topicSubscriberCallback, this, ros::TransportHints().tcpNoDelay());
  timer_fast_       = nh_.createTimer(ros::Rate(100.0), &TimerTester::timerFast, this);
  timer_slow_       = nh_.createTimer(ros::Rate(1.0), &TimerTester::timerSlow, this);
#endif

  ROS_INFO_ONCE("[TimerTester]: initialized");
}

// | ------------------------ callbacks ----------------------- |

void TimerTester::timerFast([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO_THROTTLE(0.1, "[TimerTester]: 100 Hz timer spinning");
}

void TimerTester::timerSlow([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO_THROTTLE(0.1, "[TimerTester]: 1 Hz timer spinning");
}

void TimerTester::callbackDrs([[maybe_unused]] timer_tester::timer_testerConfig& config, [[maybe_unused]] uint32_t level) {

  ROS_INFO("[TimerTester]: callbackDrs() called");
}

void TimerTester::topicSubscriberCallback([[maybe_unused]] const std_msgs::EmptyConstPtr& msg) {

  ROS_INFO_THROTTLE(0.1, "[TimerTester]: topic callback alive");
}

}  // namespace timer_tester

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(timer_tester::TimerTester, nodelet::Nodelet);

# ros::Timer + dynamic_reconfigure_server 20.04 bug

TL;DR: ROS timers don't start if initialized before a dynamic reconfigure server.

## Environment

* Ubuntu 20.04 + ROS Noetic
* empty workspace with no specific flags
* tested on amd64 or arm64

## Observations

* when **ros::Timer** is initialized before a **dynamic reconfigure server** (DRS), the timer won't start
* it affects fast timers (100 Hz and more) rather than slow timers
* it is non-deterministic, sometimes it starts correctly
* the higher the number of DRS parameters, the higher the chance of replicating the issue (10+ parameters = near 100% chance)
* seems not to be influenced by the computational resources, happens on i9-9900K as well as on Rpi4
* does not happen at all on 18.04 + ROS Melodic

## Minimal non-working example

* has two timers, 1 Hz and 100 Hz
* has a pre-compiler `#define` which can switch the order of initialization
* the 100 Hz timer won't run

## Solution

* I don't know, the DRS seems to be devoid of ros::Timers.
* Can't say for sure, but swapping the order is not a viable workaround for ros::Pluginlib plugins. So far it looks like some plugins' timers are blocked by other plugins' DRSs.
* the only workaround I found is to use a slow timer (or a thread) to check the activity and restart any broken fast timers... but... duh...


#include "std_msgs/String.h"
#include "ros_subscribe.hh"
#include "cadac.h"

void numchatterCallback(const trick_msgs::cadac::ConstPtr& msg) {
  std::cout << *msg << std::endl ;
}

RosSubscribe::RosSubscribe() {}

int RosSubscribe::init() {
    sub = n.subscribe("cadac_bus", 1000, numchatterCallback) ;
    return 0 ;
}

int RosSubscribe::process() {
    ros::spinOnce();
    return 0 ;
}


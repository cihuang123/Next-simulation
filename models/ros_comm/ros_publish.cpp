
#include "std_msgs/String.h"
#include "trick/exec_proto.h"
#include "ros_publish.hh"

#include "cadac.h"

RosPublish::RosPublish() : count(0) {}

int RosPublish::init() {
    msg_pub = n.advertise<trick_msgs::cadac>("cadac_bus", 1000) ;
    return 0 ;
}

int RosPublish::publish() {
    if ( ros::ok() ) {
        trick_msgs::cadac tn ;
        count++;
        tn.first_name = "Dung-Ru" ;
        tn.last_name = "Tsai" ;
        tn.date = "April 4, 1989" ;
        tn.points = 18 ;
        tn.rebounds = 16 ;
        tn.blocks = 11 ;
        tn.assists = count ;
        //std::cout << tn << std::endl ;
        msg_pub.publish(tn) ;

        ros::spinOnce();
    } else {
        exec_terminate(__FILE__, "ros not ok") ;
    }
    return 0 ;
}


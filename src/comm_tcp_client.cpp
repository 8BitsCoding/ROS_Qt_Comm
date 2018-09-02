#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros_tutorial_comm/MsgTutorial.h"

// TCP Client Include
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

void chatterCallback( const ros_tutorial_comm::MsgTutorial::ConstPtr& msg)
{
  ROS_INFO("receive msg = %d", msg->stamp.sec);
  ROS_INFO("receive msg = %d", msg->stamp.nsec);
  ROS_INFO("receive msg = %d", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "comm_tcp_client");
  ros::NodeHandle nh;

  ros::Subscriber ros_tutorials_sub = nh.subscribe("comm_ros_tutorial_msg", 100, chatterCallback);

  ros::spin();

  return 0;
}

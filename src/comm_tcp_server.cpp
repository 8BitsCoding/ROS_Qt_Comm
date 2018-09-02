#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros_tutorial_comm/MsgTutorial.h"

// TCP comm include
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

using namespace std;

// Error Report
void error(const char *msg) {
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
  // ROS topic Pub Param
  ros::init(argc, argv, "comm_tcp_server");
  ros::NodeHandle nh;

  ros::Publisher ros_tutorial_pub = nh.advertise<ros_tutorial_comm::MsgTutorial>("comm_ros_tutorial_msg", 100);
  ros::Rate loop_rate(10);
  ros_tutorial_comm::MsgTutorial msg;
  int count = 0;

  // TCP Param
  int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;

  std_msgs::String message;
  std::stringstream ss;

  // port num check
  if (argc < 2) {
    fprintf(stderr,"ERROR, no port provided\n");
    exit(1);
  }

  portno = atoi(argv[1]);
  cout << "Hello there! This node is listening on port " << portno << " for incoming connections" << endl;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
      error("ERROR opening socket");

  int enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
      error("setsockopt(SO_REUSEADDR) failed");

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);
  if (bind(sockfd, (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)) < 0)
            error("ERROR on binding");
  listen(sockfd,5);
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd,
              (struct sockaddr *) &cli_addr,
              &clilen);
  if (newsockfd < 0)
       error("ERROR on accept");

  while (ros::ok())
  {
    msg.stamp = ros::Time::now();
    msg.data = count;

    ROS_INFO("send msg = %d", msg.stamp.sec);
    ROS_INFO("send msg = %d", msg.stamp.nsec);
    ROS_INFO("send msg = %d", msg.data);

    ros_tutorial_pub.publish(msg);


    loop_rate.sleep();

    ++count;

    //////////////////////////////////////////

    ss.str(std::string()); //Clear contents of string stream
    bzero(buffer,256);

    n = read(newsockfd,buffer,255);

    if (n < 0) error("ERROR reading from socket");
    // printf("Here is the message: %s\n",buffer);
    ss << buffer;

    message.data = ss.str();

    ROS_INFO("%s", message.data.c_str());

    server_pub.publish(message);

    n = write(newsockfd,"I got your message",18);

    if (n < 0) error("ERROR writing to socket");
  }

  return 0;
}

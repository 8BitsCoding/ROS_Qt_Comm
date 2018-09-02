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

#include "std_msgs/String.h"

#define MESSAGE_FREQ 1

class EthernetManage {
public:
  void DisConnection();
  int Connection();
  bool Send_Data();
  static void Receive_data(void* arg);

private:
  // tcp 통신 파라미터 세팅
  int sockfd, portno, n, choice = 1;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[256] = "\0";
  bool echoMode = false;
}

void EthernetManage::Connection()
{
	sockfd = socket(AF_INET, SOCK_STREAM, 0);

  // 소킷 할당 확인
  if (sockfd < 0) 
    error("ERROR opening socket");
	
  // 소켓 초기화
  bzero((char *) &serv_addr, sizeof(serv_addr));

  // 주소할당
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(portno);

  // 서버에 connect 시도
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
    error("ERROR connecting");
}

void EthernetManage::DisConnection()
{
	closesocket(sockfd);
}


/*
// 알아두면 좋을 듯 하여 복사해둠. 참고하시오.
class Listener {
private:
    char topic_message[256] = { 0 };
public:
    void callback(const std_msgs::String::ConstPtr& msg);
    char* getMessageValue();
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    ROS_INFO("I heard:[%s]", msg->data.c_str());
}

char* Listener::getMessageValue() {
    return topic_message;
}
*/

// 에러처리
void error(const char *msg) {
    perror(msg);
    exit(0);
}


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

  // ip port 번호 들
  if (argc < 3) {
      fprintf(stderr,"Usage: $ rosrun comm_tcp client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
      exit(0);
  }

  // 에코모드 확인
  if (argc > 3)
		if (strcmp(argv[3], "-e") == 0)
			m_EthManage.echoMode = true;

  EthernetManage m_EthManage;

  // 서버 ip주소 할당
  m_EthManage.server = gethostbyname(argv[1]);

  // 포트번호 할당 및 소킷 할당
  m_EthManage.portno = atoi(argv[2]);

  while(ros::ok()) {
    // 버퍼를 비우고
    bzero(m_EthManage.buffer,256);

    // 보낼 문자열 입력
    printf("Please enter the message : ");
    fgets(m_EthManage.buffer, 255, stdin);

    n = write(m_EthManage.sockfd, m_EthManage.buffer ,strlen(m_EthManage.buffer));

    if(n < 0)
      error("ERROR writing to socket");

    if(m_EthManage.echoMode) {
      bzero(m_EthManage.buffer, 256);
      m_EthManage.n = read(m_EthManage.sockfd, m_EthManage.buffer ,255);
      if(m_EthManage.n < 0)
      {
        m_EthManage.DisConnection();
        m_EthManage.Connection();
        //error("ERROR reading reply");
      }

      printf("%s\n", m_EthManage.buffer);
    }

    ros::spinOnce();
  }

  //ros::spin();

  return 0;
}



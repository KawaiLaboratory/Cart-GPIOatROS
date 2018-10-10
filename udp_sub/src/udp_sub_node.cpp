#include "ros/ros.h"
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "udp_sub");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  int sock;
  struct sockaddr_in addr;

  char buf[2048];
  char* delim = ",";
  char* ctx;

  sock = socket(AF_INET, SOCK_DGRAM, 0);

  addr.sin_family = AF_INET;
  addr.sin_port = htons(12345);
  addr.sin_addr.s_addr = INADDR_ANY;

  while(ros::ok()){
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    memset(buf, 0, sizeof(buf));
    recv(sock, buf, sizeof(buf), 0);

    char* l_char = strtok_r(buf, delim, &ctx);
    char* theta_char = strtok_r(NULL, delim, &ctx);

    printf("%s,%s\n", l_char, theta_char);

    close(sock);
    loop_rate.sleep();
  }

 return 0;
}

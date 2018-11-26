#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Bool.h"

#define SHUTDOWN_PIN 17
#define CLUTCH_PIN   27
#define START_PIN    22
#define SETUP_LED    26
#define DRIVING_LED  12

int pi;
extern int pi;

int main(int argc, char **argv){
  ros::init(argc, argv, "led_test");
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Rate loop_rate(20);

  pub = n.advertise<std_msgs::Bool>("driving_flag", 1000);

  pi = pigpio_start("localhost", "8888");
  set_mode(pi, SHUTDOWN_PIN, PI_INPUT);
  set_mode(pi, CLUTCH_PIN, PI_INPUT);
  set_mode(pi, START_PIN, PI_INPUT);
  set_mode(pi, SETUP_LED, PI_OUTPUT);
  set_mode(pi, DRIVING_LED, PI_OUTPUT);

  bool ClutchFlg   = false;
  bool ShutdownFlg = false;
  bool InitFlg     = false;
  bool DrivingFlg  = false;

  int clutch_status;
  int shutdown_status;

  while(ros::ok()){
    std_msgs::Bool flag;
    clutch_status   = gpio_read(pi, CLUTCH_PIN);
    shutdown_status = gpio_read(pi, SHUTDOWN_PIN);

    ShutdownFlg = (shutdown_status == PI_LOW)? true : false;
    ClutchFlg   = (clutch_status   == PI_LOW)? true : false;

    InitFlg = (ClutchFlg && ShutdownFlg)? true : false;
    gpio_write(pi, SETUP_LED, InitFlg);

    if(InitFlg){
      if(wait_for_edge(pi, START_PIN, FALLING_EDGE, 1))
        DrivingFlg = !DrivingFlg;
    }

    //ROS_INFO("%d, %d, %d", ShutdownFlg, ClutchFlg, DrivingFlg);
    flag.data = DrivingFlg;
    pub.publish(flag);

    gpio_write(pi,DRIVING_LED, DrivingFlg);
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

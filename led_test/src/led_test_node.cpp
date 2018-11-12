#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"

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

  pi = pigpio_start("localhost", "8888");
  set_mode(pi, SHUTDOWN_PIN, PI_INPUT);
  set_mode(pi, CLUTCH_PIN, PI_INPUT);
  set_mode(pi, START_PIN, PI_INPUT);
  set_mode(pi, SETUP_LED, PI_OUTPUT);
  set_mode(pi, DRIVING_LED, PI_OUTPUT);

  bool ClutchFlg   = false;
  bool ShutdownFlg = false;
  bool InitFlg     = false;

  int clutch_status   = 1;
  int shutdown_status = 1;

  while(ros::ok()){
    clutch_status   = gpio_read(pi, CLUTCH_PIN);
    shutdown_status = gpio_read(pi, SHUTDOWN_PIN);

    if(shutdown_status == PI_HIGH)
      ShutdownFlg = true;
    else{
      ROS_INFO("Please check ShutdownButton");
      ShutdownFlg = false;
    }

    if(clutch_status == PI_LOW)
      ClutchFlg = true;
    else{
      ROS_INFO("Please check ClutchButton");
      ClutchFlg = false;
    }

    if(ClutchFlg && ShutdownFlg){
      InitFlg = true;
      gpio_write(pi, SETUP_LED, PI_HIGH);
    }else{
      InitFlg = false;
      gpio_write(pi, SETUP_LED, PI_LOW);
    }

    if(InitFlg){
      if(wait_for_edge(pi, START_PIN, FALLING_EDGE, 1))
	gpio_write(pi, DRIVING_LED, PI_HIGH);
      else
   	gpio_write(pi, DRIVING_LED, PI_LOW);
    }
    loop_rate.sleep();
  }
}

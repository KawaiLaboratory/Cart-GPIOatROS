#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"

#define SHUTDOWN_PIN 17
#define CLUTCH_PIN   27
#define START_PIN    22 
#define SETUP_LED    26
#define DRIVING_LED  21

int pi;
extern int pi;

bool changeMode(bool flg){
  ROS_INFO("change mode");
  return !flg;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "led_test");
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Rate loop_rate(1);

  bool ClutchFlg   = false;
  bool ShutdownFlg = false;
  bool InitFlg     = true;
  bool StartFlg    = true;

  pi = pigpio_start("localhost", "8888");
  set_mode(pi, SHUTDOWN_PIN, PI_INPUT);
  set_mode(pi, CLUTCH_PIN, PI_INPUT);
  set_mode(pi, START_PIN, PI_INPUT);
  set_mode(pi, SETUP_LED, PI_OUTPUT);
  set_mode(pi, DRIVING_LED, PI_OUTPUT);

  int clutch_status   = gpio_read(pi, CLUTCH_PIN);
  int shutdown_status = gpio_read(pi, SHUTDOWN_PIN);

  while(ros::ok()){
    /*
    if(InitFlg){
      ROS_INFO("Please push starting button");
      StartFlg = callback(pi, START_PIN, FALLING_EDGE, changeMode(StartFlg));
    }

    if(ClutchFlg && ShutdownFlg){
      InitFlg = true;
    }
      
    if(shutdown_status == PI_LOW)
      ShutdownFlg = true;
    else{
      ROS_INFO("Please check ShutdownButton");
      shutdown_status = gpio_read(pi, SHUTDOWN_PIN);
    }

    if(clutch_status == PI_LOW)
      ClutchFlg = true;
    else{
      ROS_INFO("Please check ClutchButton");
      clutch_status = gpio_read(pi, CLUTCH_PIN);
    }
    */
    if(InitFlg){
      gpio_write(pi, SETUP_LED, PI_HIGH);
    }
    if(StartFlg){
      gpio_write(pi, DRIVING_LED, PI_HIGH);
    }

    loop_rate.sleep();
  }
}

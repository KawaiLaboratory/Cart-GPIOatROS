#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"

int pi;
extern int pi;

int main(int argc, char **argv){
  ros::init(argc, argv, "led_test");
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Rate loop_rate(20);
  
  static int flash_pin[2]={38, 40};

  pi = pigpio_start("localhost", "8888");
  set_mode(pi, flash_pin[0], PI_INPUT);
  set_mode(pi, flash_pin[1], PI_INPUT);
}

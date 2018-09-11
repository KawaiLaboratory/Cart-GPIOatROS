#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "pigpiod_if2.h"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define HALF 500000 //duty比50%
#define PHASE_DIFF 1.5*M_PI //位相遅れ[rad]
#define WALK_SPEED 1.6 //[m/s]

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {21, 20};

static float d = 0.66; //タイヤ間距離[m]
static float r = 0.15; //タイヤ半径[m]
static float res = 0.0036;  //モータ分解能[deg]

static float safety = 0.3; //安全距離[m]

float distance = 0.0; //LRFから得られた最近点までの距離[m]
float angle = 0.0; //LRFから得られた最近点の角度[rad]

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int count = scan->scan_time / scan->time_increment;
  distance = 0.0;
  angle = 0.0;

  for(int i = 0; i < count; i++) {
    float rad = fmodf(scan->angle_min + PHASE_DIFF + scan->angle_increment * i, 2*M_PI);
    float degree = RAD2DEG(rad);
    if(0.0 < degree && degree < 180.0){
      if(distance == 0.0 || distance > scan->ranges[i]){
        distance = scan->ranges[i];
        angle = rad;
      }
    }
  }
  distance -= safety;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pigpio_test");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::Time begin = ros::Time::now();
  ros::Duration duration;
  ros::Subscriber sub;
  double secs = 0.0;
  float x = 0.0;
  float y = 0.0;
  bool lost = false;
  float freq = (WALK_SPEED * 90) / (M_PI * M_PI * res * r);

  pi = pigpio_start("localhost","8888");
  set_mode(pi, pwmpin[0], PI_OUTPUT);
  set_mode(pi, dirpin[0], PI_OUTPUT);
  set_mode(pi, pwmpin[1], PI_OUTPUT);
  set_mode(pi, dirpin[1], PI_OUTPUT);

  while(ros::ok() && !lost){
    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    loop_rate.sleep();
    ros::spinOnce();

    //if(distance <= 0.0){
      //ROS_INFO("Too Close");
    //}else{
      duration = ros::Time::now() - begin;
      secs = duration.toSec();
      x = distance * std::cos(angle);
      y = distance * std::sin(angle);
      ROS_INFO("x:%f, y:%f", x, y);
      ROS_INFO("%f", freq);
    //}
  }

  // 出力信号の停止
  hardware_PWM(pi, pwmpin[0], 0, 0);
  gpio_write(pi, dirpin[0], PI_LOW);
  hardware_PWM(pi, pwmpin[1], 0, 0);
  gpio_write(pi, dirpin[1], PI_LOW);

  // PINOUT -> PININ
  set_mode(pi, pwmpin[0], PI_INPUT);
  set_mode(pi, dirpin[0], PI_INPUT);
  set_mode(pi, pwmpin[1], PI_INPUT);
  set_mode(pi, dirpin[1], PI_INPUT);

  // 終了
  pigpio_stop(pi);

  return 0;
}

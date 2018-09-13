#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "pigpiod_if2.h"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DOT(x, x_prev, dt) (((x)-(x_prev))/(dt))

#define HALF 500000 //duty比50%
#define PHASE_DIFF 2*M_PI //位相遅れ[rad]
#define WALK_SPEED 1.7 //[m/s]

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {21, 20};

static float d = 0.66/2; //タイヤ間距離[m]
static float r = 0.15/2; //タイヤ半径[m]
static float b = 0.0036;  //モータ分解能[deg]

static float safety = 0.5; //安全距離[m]

float l = 0.0; //得られた最近点までの距離[m]
float theta = 0.0; //得られた最近点の角度[rad]
float theta_prev = 0.0; //直前の角度[rad]

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int count = scan->scan_time / scan->time_increment;
  l = 0.0;
  theta = 0.0;

  for(int i = 0; i < count; i++) {
    float rad = fmodf(scan->angle_min + PHASE_DIFF + scan->angle_increment * i, 2*M_PI);
    float degree = RAD2DEG(rad);
    if(0.0 < degree && degree < 180.0){
      if(l == 0.0 || l > scan->ranges[i]){
        l = scan->ranges[i];
        theta = rad;
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pigpio_test");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::Time prev = ros::Time::now();
  ros::Time now;
  ros::Duration duration;
  ros::Subscriber sub;
  double dt = 0.0;
  float x = 0.0;
  float y = 0.0;
  bool lost = false;
  double freq_l = 0.0;
  double freq_r = 0.0;
  double v = WALK_SPEED;
  double d_theta = 0.0;

  pi = pigpio_start("localhost","8888");
  set_mode(pi, pwmpin[0], PI_OUTPUT);
  set_mode(pi, dirpin[0], PI_OUTPUT);
  set_mode(pi, pwmpin[1], PI_OUTPUT);
  set_mode(pi, dirpin[1], PI_OUTPUT);

  while(ros::ok()){
    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    loop_rate.sleep();
    ros::spinOnce();

    now = ros::Time::now();
    duration = now - prev;
    dt = duration.toSec();

//    if(l <= safety){
//      ROS_INFO("Too Close");
//      hardware_PWM(pi, pwmpin[0], 0, 0);
//      hardware_PWM(pi, pwmpin[1], 0, 0);
//    }else if(!lost){
      d_theta = DOT(theta, theta_prev, dt);
      freq_l = 180.0 * (v-d*d_theta)/(M_PI*b*r);
      freq_r = 180.0 * (v+d*d_theta)/(M_PI*b*r);

      ROS_INFO("dt:%lf", dt);
      ROS_INFO("d_theta:%lf", d_theta);
      ROS_INFO("freq_l:%lf", freq_l);
      ROS_INFO("freq_r:%lf", freq_r);
      ROS_INFO("freq_l:%d", (int)freq_l);
      ROS_INFO("freq_r:%d", (int)freq_r);
      //x = distance * std::cos(angle);
      //y = distance * std::sin(angle);
      //ROS_INFO("x:%f, y:%f", x, y);
      //ROS_INFO("%f", freq);
      //gpio_write(pi, dirpin[0], PI_HIGH);
      //gpio_write(pi, dirpin[1], PI_LOW);
      //hardware_PWM(pi, pwmpin[0], (int)freq, HALF);
      //hardware_PWM(pi, pwmpin[1], (int)freq, HALF);
//    }else{
//      ROS_INFO("MADA");
//    }
    prev = now;
    theta_prev = theta;
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

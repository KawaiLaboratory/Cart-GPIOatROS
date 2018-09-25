#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "pigpiod_if2.h"
#include "math.h"

#define DOT(x, x_prev, dt) (((x)-(x_prev))/(dt))
#define RAD2DEG(x) ((x)*180./M_PI)

#define HALF 500000 //duty比50%
#define PHASE_DIFF 2*M_PI //位相遅れ[rad]
#define R 0.0036*M_PI/180.0
#define KP 0.5 //will change
#define KD 0.2 //will change

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {21, 20};

static float d = 0.66/2; //タイヤ間距離[m]
static float r = 0.15/2; //タイヤ半径[m]

static float safety = 1.0; //安全距離[m]

float l = 0.0; //得られた最近点までの距離[m]
float l_prev = 0.0;
float theta = 0.0; //得られた最近点の角度[rad]
float theta_prev = 0.0;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int count = scan->scan_time / scan->time_increment;
  l = 0.0;
  theta = 0.0;
  for(int i = 0; i < count; i++) {
    float rad = fmodf(scan->angle_min + PHASE_DIFF + scan->angle_increment * i, 2*M_PI);
    float degree = RAD2DEG(rad);
    if(90.0 < degree  && degree < 270.0){
      if(i == 0 || l > scan->ranges[i]){
        l = scan->ranges[i];
        theta = rad;
      }
    }
  }
  if(l_prev == 0.0 && theta_prev == 0.0){
    l_prev = l;
    theta_prev = theta;
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

  bool lost = false;

  double dt = 0.0;
  double u_l = 0.0;
  double u_r = 0.0;
  double v = 0.0;
  double ohm = 0.0;

  double d_theta = 0.0;
  double d_l = 0.0;

  double k_pl = KP/(R*r);
  double k_dl = KD/(R*r);
  double k_pt = KP*d/(R*r);
  double k_dt = KD*d/(R*r);

  double Ul = 0.0;
  double Ut = 0.0;

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

    if(!lost){
      d_theta = DOT(theta, theta_prev, dt);
      d_l     = DOT(l, l_prev, dt);

      Ul = k_pl * ( l - 1 ) + k_dl * ( d_l - v );
      Ut = k_pt * ( theta - M_PI ) + k_dt * ( d_theta - ohm );

      u_r = Ul + Ut;
      u_l = Ul - Ut;

      v   = R * r / 2 * ( u_r + u_l);
      ohm = R * r / ( 2*d ) * ( u_r - u_l);

      ROS_INFO("dt:%lf", dt);
      ROS_INFO("u_l:%d", (int)u_l);
      ROS_INFO("u_r:%d", (int)u_r);
      ROS_INFO("v:%lf", v);
      ROS_INFO("ohm:%lf", ohm);

      if(u_r < 0){
        gpio_write(pi, dirpin[0], PI_LOW);
        u_r = std::fabs(u_r);
      }else{
        gpio_write(pi, dirpin[0], PI_HIGH);
      }

      if(u_l < 0){
        gpio_write(pi, dirpin[1], PI_HIGH);
        u_l = std::fabs(u_l);
      }else{
        gpio_write(pi, dirpin[1], PI_LOW);
      }

      hardware_PWM(pi, pwmpin[0], (int)u_r, HALF);
      hardware_PWM(pi, pwmpin[1], (int)u_l, HALF);
    //}else{
    //  ROS_INFO("soon");
    }

    theta_prev = theta;
    l_prev = l;
    prev = now;
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

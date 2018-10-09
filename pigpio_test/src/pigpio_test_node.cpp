#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "pigpiod_if2.h"
#include "math.h"

#define DOT(x, x_prev, dt) (((x)-(x_prev))/(dt))
#define RAD2DEG(x) ((x)*180./M_PI)

#define HALF 500000 //duty比50%
#define PHASE_DIFF 2*M_PI //位相遅れ[rad]
#define R 0.0036*M_PI/180.0

#define KP 0.3
#define KD 0.1

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {21, 20};

static float d = 0.66/2; //タイヤ間距離[m]
static float r = 0.15/2; //タイヤ半径[m]

double l = 0.0;     double l_prev = 0.0; //得られた最近点までの距離[m]
double theta = 0.0; double theta_prev = 0.0;//得られた最近点の角度[rad]

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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pigpio_test");
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Rate loop_rate(5);

  pi = pigpio_start("localhost","8888");
  set_mode(pi, pwmpin[0], PI_OUTPUT);
  set_mode(pi, dirpin[0], PI_OUTPUT);
  set_mode(pi, pwmpin[1], PI_OUTPUT);
  set_mode(pi, dirpin[1], PI_OUTPUT);

  bool lost = false; //find target?
  double dt = 0.0;   //control cycle

  double u_l = 0.0;  double u_r = 0.0;  //input left hz & right hz
  double v = 0.0;    double ohm = 0.0;  //cart's output speed
  double dx_c = 0.0; double dy_c = 0.0; //cart's x speed & y speed

  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

  loop_rate.sleep();
  ros::spinOnce();

  double x_pprev =  l * std::sin(theta); double dx_p = 0.0; //point's x prev position & speed
  double y_pprev = -l * std::cos(theta); double dy_p = 0.0; //point's y prev position & speed

  double x_p = 0.0; //point's current x position
  double y_p = 0.0; //point's current y position
  double alpha = 0.0;

  double e_x=0.0; double e_y=0.0; double e_xv=0.0; double e_yv=0.0; //position & speed error

  double X_e = 0.0;     double Y_e = 0.0; //error sum
  double tmpSqrt = 0.0; //tmp num

  ros::Time prev = ros::Time::now();
  ros::Time now;
  ros::Duration duration;

  while(ros::ok()){
    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    loop_rate.sleep();
    ros::spinOnce();

    now = ros::Time::now();
    duration = now - prev;
    dt = duration.toSec();
    prev = ros::Time::now();

    ROS_INFO("%lf", dt);
    if(!lost){
      x_p =  l * std::sin(theta);
      y_p = -l * std::cos(theta);
      alpha = theta - M_PI;

      dx_p = (x_p - x_pprev)/dt;
      dy_p = (y_p - y_pprev)/dt;

      ROS_WARN("===================================");
      ROS_INFO("xprev:%lf, yprev:%lf", x_pprev, y_pprev);
      ROS_INFO("x_p:%lf, y_p:%lf", x_p, y_p);
      ROS_INFO("dxp:%lf, dyp:%lf", dx_p, dy_p);

      if(0.25<x_p*x_p){
        e_x  = x_p;
        e_xv = dx_p - dx_c;
      }else{
        e_x  = -0.0;
        e_xv = -0.0;
      }

      if(1.0<y_p){
        e_y  = y_p - 0.5;
        e_yv = dy_p - dy_c;
      }else{
        e_y  = 0.0;
        e_yv = 0.0;
      }

      X_e = e_x + e_xv;
      Y_e = e_y + e_yv;

      tmpSqrt = std::sqrt(X_e*X_e+Y_e*Y_e);

      u_r = (1+2*d*std::sin(alpha)/l)*tmpSqrt/(R*r);
      u_l = (1-2*d*std::sin(alpha)/l)*tmpSqrt/(R*r);

      if(u_r < 0){
        gpio_write(pi, dirpin[0], PI_HIGH);
        u_r = std::fabs(u_r);
      }else{
        gpio_write(pi, dirpin[0], PI_LOW);
      }

      if(u_l < 0){
        gpio_write(pi, dirpin[1], PI_LOW);
        u_l = std::fabs(u_l);
      }else{
        gpio_write(pi, dirpin[1], PI_HIGH);
      }

      hardware_PWM(pi, pwmpin[0], (int)u_l, HALF);
      hardware_PWM(pi, pwmpin[1], (int)u_r, HALF);

      v   = R*r/2*(u_r + u_l);
      ohm = R*r/(2*d)*(u_r - u_l);

      dx_c = v*std::cos(ohm*dt+M_PI);
      dy_c = v*std::sin(ohm*dt+M_PI);
    }
    x_pprev = x_p;
    y_pprev = y_p;
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

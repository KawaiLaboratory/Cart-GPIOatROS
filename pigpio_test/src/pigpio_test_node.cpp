#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "pigpiod_if2.h"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DOT(x, x_prev, dt) (((x)-(x_prev))/(dt))

#define HALF 500000 //duty比50%
#define PHASE_DIFF 1.5*M_PI //位相遅れ[rad]
#define KPl 1 //will change
#define KDl 1 //will change
#define KPt 1 //will change
#define KDt 1 //will change

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {21, 20};

static float d = 0.66/2; //タイヤ間距離[m]
static float r = 0.15/2; //タイヤ半径[m]
static float b = 0.0036;  //モータ分解能[deg]

static float safety = 0.5; //安全距離[m]

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
    float degree = RAD2DEG(fmodf(scan->angle_min + PHASE_DIFF + scan->angle_increment * i, 2*M_PI));
    if(0.0 < degree && degree < 180.0){
      if(i == 0 || l > scan->ranges[i]){
        l = scan->ranges[i];
        theta = degree;
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
  bool lost = false;
  double u_l = 0.0;
  double u_r = 0.0;
  double v = 0.0;
  double ohm = 0.0;
  
  double d_theta = 0.0;
  double d_l = 0.0;

  pi = pigpio_start("localhost","8888");
  set_mode(pi, pwmpin[0], PI_OUTPUT);
  set_mode(pi, dirpin[0], PI_OUTPUT);
  set_mode(pi, pwmpin[1], PI_OUTPUT);
  set_mode(pi, dirpin[1], PI_OUTPUT);

  while(ros::ok()){
    now = ros::Time::now();
    duration = now - prev;
    dt = duration.toSec();
    
    //if(l <= safety){
      //ROS_INFO("Too Close");
      //hardware_PWM(pi, pwmpin[0], 0, 0);
      //hardware_PWM(pi, pwmpin[1], 0, 0);
    //}else if(!lost){
         
      d_theta = DOT(theta, theta_prev, dt);
      d_l     = DOT(l, l_prev, dt);
      
      v   = M_PI * b * r / 360 * ( u_r + u_l/d);
      ohm = M_PI * b * r / 360 * ( u_r - u_l/d);
      
      u_r = ( KPl * ( l - 1 ) + KDl * ( d_l - v ) + KPt * ( theta - M_PI/2 ) + KDt * ( d_theta - ohm ))*1/2;
      u_l = ( KPl * ( l - 1 ) + KDl * ( d_l - v ) - KPt * ( theta - M_PI/2 ) - KDt * ( d_theta - ohm ))*d/2;
      
      ROS_INFO("dt:%lf", dt);
      ROS_INFO("u_l:%lf", u_l);
      ROS_INFO("u_r:%lf", u_r);
      ROS_INFO("v:%lf", v);
      ROS_INFO("ohm:%lf", ohm);
      
      if(u_r < 0){
        gpio_write(pi, dirpin[0], PI_LOW);
      }else{
        gpio_write(pi, dirpin[0], PI_HIGH);
      }
      
      if(u_l < 0){
        gpio_write(pi, dirpin[1], PI_HIGH);
      }else{
        gpio_write(pi, dirpin[1], PI_LOW);
      }
      
      hardware_PWM(pi, pwmpin[0], (int)u_r, HALF);
      hardware_PWM(pi, pwmpin[1], (int)u_l, HALF);
    //}else{
    //  ROS_INFO("mada");
    //}
    
    theta_prev = theta;
    l_prev = l;
    
    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    prev = now;
    loop_rate.sleep();
    ros::spinOnce();
    
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

#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "fstream"

using namespace std;

int pi;
extern int pi;

class CBFController{
  private:
    double u_v  = 0.0;
    double u_om = 0.0;
  public:
    void Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
      u_v  = msg->data[0];
      u_om = msg->data[1];
    };
    double get_u_v(){
      return u_v;
    };
    double get_u_om(){
      return u_om;
    };
};

class Cartbot{
  private:
    double x  = 0;
    double y  = 0;
    double th = 0;
    double v  = 0;
    double om = 0;
  public:
    void update(double u_v, double u_om, double dt){
      x  = x + u_v * dt * cos(th);
      y  = y + u_v * dt * sin(th);
      th = th + u_om*dt;
      v  = u_v;
      om = u_om;
      ROS_INFO("x:%.2f, y:%.2f, th:%.2f", x, y, th);
    };
};

class Serial{
  private:
    const int pin_pwm[2] = {18, 19};
    const int pin_dir[2] = {21, 20};
    const int HALF = 500000;
    int u_r_in = 0;
    int u_l_in = 0;
  public:
    Serial(){
      pi = pigpio_start("localhost","8888");
      for (int i = 0; i < 2; i++){
        set_mode(pi, pin_pwm[i], PI_OUTPUT);
        set_mode(pi, pin_dir[i], PI_OUTPUT);
      }
    };
    ~Serial(){
      for (int i = 0; i < 2; i++){
        hardware_PWM(pi, pin_pwm[i], 0, 0);
        set_mode(pi, pin_pwm[i], PI_INPUT);
        gpio_write(pi, pin_dir[i], PI_LOW);
        set_mode(pi, pin_dir[i], PI_INPUT);
      }
      pigpio_stop(pi);
    };
    void input(int u_r, int u_l){
      if(u_r < 0){
        gpio_write(pi, pin_dir[0], PI_LOW);  // タイヤの回転方向指定
        u_r_in = abs(u_r);                  // 入力周波数指定
      }else{
        gpio_write(pi, pin_dir[0], PI_HIGH); // タイヤの回転方向指定
        u_r_in = u_r;                       // 入力周波数指定
      }
      if(u_l < 0){
        gpio_write(pi, pin_dir[1], PI_HIGH); // タイヤの回転方向指定
        u_l_in = abs(u_l);                  // 入力周波数指定
      }else{
        gpio_write(pi, pin_dir[1], PI_LOW);  // タイヤの回転方向指定
        u_l_in = u_l;                       // 入力周波数指定
      }

      hardware_PWM(pi, pin_pwm[0], u_r_in, HALF);
      hardware_PWM(pi, pin_pwm[1], u_l_in, HALF);
    };
};

class Controller{
  private:
    /* ゲイン*/
    const float K1  = 1;
    const float K2  = 1;
    /* カート本体のパラメータ */
    const double R    = 0.0036*M_PI/180.0;
    const float  r    = 0.15/2;
    const float  T    = 0.66;
    /* debugmode */
    const bool debug_flg = true;
    /* カート */
    Cartbot cart;
    /* シリアル */
    Serial ser;
    /* PWM入力 */
    int u_r  = 0;
    int u_l  = 0;
    /* PWMによる速度 */
    double v  = 0.0;
    double om = 0.0;
  public:
    void run(double u_v, double u_om, double dt){
      if(debug_flg){
        cart.update(v, om, dt);
      }

      u_r = int((2*u_v+T*u_om)/(2*R*r));
      u_l = int((2*u_v-T*u_om)/(2*R*r));

      v  = R*r/2*(u_r+u_l);
      om = R*r/T*(u_r-u_l);

      ser.input(u_r, u_l);
    };
};

int main(int argc, char **argv){
  ros::init(argc, argv, "feedback");
  ros::NodeHandle n;

  ros::Rate rate(20);

  ros::Time prev;
  ros::Time now;

  double dt   = 0;
  double u_v  = 0;
  double u_om = 0;

  Controller    c;
  CBFController cbf;

  ros::Subscriber sub = n.subscribe("/u_cbf", 2, &CBFController::Callback, &cbf);

  prev = ros::Time::now();
  rate.sleep();

  while(ros::ok()){
    now = ros::Time::now();
    dt = (now-prev).toSec();

    u_v  = cbf.get_u_v();
    u_om = cbf.get_u_om();

    c.run(u_v, u_om, dt);

    ros::spinOnce();
    rate.sleep();
    prev = now;
  }
  return 0;
}

#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "fstream"

using namespace std;

int pi;
extern int pi;

class LRF{
  private:
    bool   lost = false;
    double x_d  = 0.0;
    double y_d  = 0.0;
    double phi  = 0.0;
  public:
    void PointCallback(const std_msgs::Float32MultiArray::ConstPtr& status){
      lost = (status->data[0] != 0.0);
      if(!lost){
        x_d = status->data[1];
        y_d = status->data[2];
      }
      phi = atan2(y_d, x_d);
    };
    double get_x_d(){
      return x_d;
    };
    double get_y_d(){
      return y_d;
    };
    double get_phi(){
      return phi;
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
      x  = x + u_v * dt * cos(th+u_om*dt/2);
      y  = y + u_v * dt * sin(th+u_om*dt/2);
      th = th + u_om*dt;
      v  = u_v;
      om = u_om;
    };
};

class Serial{
  private:
    const int pin_pwm[2] = {18, 19};
    const int pin_dir[2] = {20, 21};
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
        gpio_write(pi, pin_dir[0], PI_HIGH);  // タイヤの回転方向指定
        u_r_in = abs(u_r);                  // 入力周波数指定
      }else{
        gpio_write(pi, pin_dir[0], PI_LOW); // タイヤの回転方向指定
        u_r_in = u_r;                       // 入力周波数指定
      }
      if(u_l < 0){
        gpio_write(pi, pin_dir[1], PI_LOW); // タイヤの回転方向指定
        u_l_in = abs(u_l);                  // 入力周波数指定
      }else{
        gpio_write(pi, pin_dir[1], PI_HIGH);  // タイヤの回転方向指定
        u_l_in = u_l;                       // 入力周波数指定
      }

      hardware_PWM(pi, pin_pwm[0], u_r_in, HALF);
      hardware_PWM(pi, pin_pwm[1], u_l_in, HALF);
    };
};

class Controller{
  private:
    /* ゲイン*/
    const float K1  = 0.5;
    const float K2  = 1;
    /* カート本体のパラメータ */
    const double R    = 0.0036*M_PI/180.0;
    const float  r    = 0.15/2;
    const float  T    = 0.66;
    /* saturation */
    const float MAX_V  = 1;
    const float MAX_OM = 1;
    /* debugmode */
    const bool debug_flg = true;
    /* カート */
    Cartbot cart;
    /* シリアル */
    Serial ser;
    /* 制御則による速度入力 */
    double u_v  = 0.0;
    double u_om = 0.0;
    /* PWM入力 */
    int u_r  = 0;
    int u_l  = 0;
    /* PWMによる速度 */
    double v  = 0.0;
    double om = 0.0;
  public:
    void run(double x_e, double y_e, double phi, double dt){
      cart.update(v, om, dt);

      u_v  = K1*x_e;
      u_om = K2*phi+K1*sin(phi)*cos(phi);

      if(u_v < -MAX_V){
        u_v = -MAX_V;
      }else if(u_v > MAX_V){
        u_v = MAX_V;
      }
      if(u_om < -MAX_OM){
        u_om = -MAX_OM;
      }else if(u_om > MAX_OM){
        u_om = MAX_OM;
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

  double dt = 0;
  double x_e = 0;
  double y_e = 0;
  double phi = 0;

  Controller c;
  LRF        lrf;

  ros::Subscriber sub = n.subscribe("/status", 1000, &LRF::PointCallback, &lrf);

  prev = ros::Time::now();
  rate.sleep();

  while(ros::ok()){
    now = ros::Time::now();
    dt = (now-prev).toSec();

    x_e = lrf.get_x_d();
    y_e = lrf.get_y_d();
    phi = lrf.get_phi();

    c.run(x_e, y_e, phi, dt);

    ros::spinOnce();
    rate.sleep();
    prev = now;
  }
  return 0;
}

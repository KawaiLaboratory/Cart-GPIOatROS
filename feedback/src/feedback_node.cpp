#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "fstream"
#include "tuple"

using namespace std;

class Cartbot{
  private:
    double x  = 0;
    double y  = 0;
    double th = M_PI/2;
    double v  = 0;
    double om = 0;
  public:
    tuple<double, double, double, double, double> update(double u_v, double u_om, double dt){
      x  = x + u_v * dt * cos(th+u_om*dt/2);
      y  = y + u_v * dt * sin(th+u_om*dt/2);
      th = th + u_om*dt;
      v  = u_v;
      om = u_om;

      return {x, y, th, v, om};
    };
};

class Serial{
  private:
    int pi;
    extern int pi;
    const  int pin_pwm = {18, 19};
    const  int pin_dir = {20, 21};
    int u_r_in = 0;
    int u_l_in = 0;
  public:
    Serial(){
      pi = pigpio_start("localhost","8888");
      for (int i = 0; i < 2; i++){
        set_mode(pi, pwmpin[i], PI_OUTPUT);
        set_mode(pi, dirpin[i], PI_OUTPUT);
      }
    };
    ~Serial(){
      for (int i = 0; i < 2; i++){
        hardware_PWM(pi, pwmpin[i], 0, 0);
        set_mode(pi, pwmpin[i], PI_INPUT);
        gpio_write(pi, dirpin[i], PI_LOW);
        set_mode(pi, dirpin[i], PI_INPUT);
      }
      pigpio_stop(pi);
    };
    void input(int u_r, int u_l){
      if(u_r < 0){
        gpio_write(pi, dirpin[0], PI_LOW);  // タイヤの回転方向指定
        u_r_in = abs(u_r);                  // 入力周波数指定
      }else{
        gpio_write(pi, dirpin[0], PI_HIGH); // タイヤの回転方向指定
        u_r_in = u_r;                       // 入力周波数指定
      }
      if(u_l < 0){
        gpio_write(pi, dirpin[1], PI_HIGH); // タイヤの回転方向指定
        u_l_in = abs(u_l);                  // 入力周波数指定
      }else{
        gpio_write(pi, dirpin[1], PI_LOW);  // タイヤの回転方向指定
        u_l_in = u_l;                       // 入力周波数指定
      }

      hardware_PWM(pi, pwmpin[0], u_l_in, HALF);
      hardware_PWM(pi, pwmpin[1], u_r_in, HALF);
    };
};

class Controller{
  private:
    /* ゲイン*/
    const float Kx  = 2;
    const float Ky  = 2;
    const float Kth = 2;
    /* 目標速度及び回転速度 */
    const float v_r  = 0.01;
    const float om_r = 0.01;
    /* カート本体のパラメータ */
    const int    HALF = 500000;
    const double R    = 0.0036*M_PI/180.0;
    const float  r    = 0.15/2;
    const float  T    = 0.66;
    /* カート */
    Cartbot cart;
    /* シリアル */
    Serial ser;
    /* 速度入力 */
    double u_v  = 0.0;
    double u_om = 0.0;
    /* PWM入力 */
    int u_r  = 0;
    int u_l  = 0;
    /* 偏差 */
    double x_e  = 0.0;
    double y_e  = 0.0;
    double th_e = 0.0;
  public:
    int run(double x_r, double y_r, double th_r, double dt){
      auto status = cart.update(u_v, u_om, 0.05);
      double x  = get<0>(status);
      double y  = get<1>(status);
      double th = get<2>(status);

      x_e  = (x_r-x)*cos(th) + (y_r-y)*sin(th);
      y_e  = (y_r-y)*cos(th) - (x_r-x)*sin(th);
      th_e = th_r-th;

      u_v  = v_r*cos(th_e) + Kx*x_e;
      u_om = om_r + Ky*y_e*v_r + Kth*sin(th_e);

      u_r = int((2*u_v+T*u_om)/(2*R*r));
      u_l = int((2*u_v-T*u_om)/(2*R*r));

      return 0;
    };
};

int main(int argc, char **argv){
  ros::init(argc, argv, "feedback");
  ros::NodeHandle n;

  ros::Rate rate(20);

  ros::Time prev;
  ros::Time now;

  double x_r = 2;
  double y_r = 2;
  double th_r = M_PI/4;

  Controller c;
}


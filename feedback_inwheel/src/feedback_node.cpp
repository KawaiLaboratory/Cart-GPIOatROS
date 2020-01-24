#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "fstream"
#include "tuple"

using namespace std;

int pi;
extern int pi;

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
    const int pin_pwm[2] = {12, 13};
    const int pin_dir[2] = {23, 24};
    const int pin_hs[6]  = {7, 8, 16, 20, 21, 25};
    const int FREQ       = 10000;
    int u_r_in = 0;
    int u_l_in = 0;
    ros::Time r_prev = ros::Time::now();
    ros::Time l_prev = ros::Time::now();
    ros::Time r_now  = r_prev;
    ros::Time l_now  = l_prev;
    double    r_dt   = 0.0;
    double    l_dt   = 0.0;
  public:
    Serial(){
      pi = pigpio_start("localhost","8888");
      for (int i = 0; i < 2; i++){
        set_mode(pi, pin_pwm[i], PI_OUTPUT);
        set_mode(pi, pin_dir[i], PI_OUTPUT);
      }
      for (int i = 0; i < 6; i++){
        set_mode(pi, pin_hs[i], PI_INPUT);
        if(i < 3){
          callback(pi, pin_hs[i], FALLING_EDGE, read_r());
        }else{
          callback(pi, pin_hs[i], FALLING_EDGE, read_l());
        }
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

      hardware_PWM(pi, pin_pwm[0], FREQ, u_r_in);
      hardware_PWM(pi, pin_pwm[1], FREQ, u_l_in);
    };
    void read_r(){
      r_now  = ros::Time::now();
      r_dt   = (r_now - r_prev).toSec();
      r_prev = r_now;
    };
    void read_l(){
      l_now  = ros::Time::now();
      l_dt   = (l_now - l_prev).toSec();
      l_prev = l_now;
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
    const float  r = 0.17/2; // タイヤ半径
    const float  T = 0.61;   // トレッド
    /* saturation */
    const int MAX_DUTY = 1000000;
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
    /* ｾﾝｻから得られた速度 */
    double v_hs  = 0.0;
    double om_hs = 0.0;
    /* 偏差 */
    double x_e  = 0.0;
    double y_e  = 0.0;
    double th_e = 0.0;
  public:
    void run(double x_r, double y_r, double th_r, double dt){
      auto status = cart.update(u_v, u_om, dt);
      double x  = get<0>(status);
      double y  = get<1>(status);
      double th = get<2>(status);

      x_e  = (x_r-x)*cos(th) + (y_r-y)*sin(th);
      y_e  = (y_r-y)*cos(th) - (x_r-x)*sin(th);
      th_e = th_r-th;

      u_v  = v_r*cos(th_e) + Kx*x_e;
      u_om = om_r + Ky*y_e*v_r + Kth*sin(th_e);

      u_r = int((2*u_v+T*u_om)*(MAX_DUTY/11));
      u_l = int((2*u_v-T*u_om)*(MAX_DUTY/11));

      if(u_r < -MAX_DUTY){
        u_r = -MAX_DUTY;
      }else if(u_r > MAX_DUTY){
        u_r = MAX_DUTY;
      }
      if(u_l < -MAX_DUTY){
        u_l = -MAX_DUTY;
      }else if(u_l > MAX_DUTY){
        u_l = MAX_DUTY;
      }

      ser.input(u_r, u_l);

    };
};

int main(int argc, char **argv){
  ros::init(argc, argv, "feedback_inwheel");
  ros::NodeHandle n;

  ros::Rate rate(20);

  ros::Time prev = ros::Time::now();;
  ros::Time now  = prev;

  double x_r  = 2;
  double y_r  = 2;
  double th_r = M_PI/4;
  double dt   = 0;

  Controller c;

  rate.sleep();

  while(ros::ok()){
    now = ros::Time::now();
    dt = (now-prev).toSec();

    c.run(x_r, y_r, th_r, dt);

    ros::spinOnce();
    rate.sleep();
    prev = now;
  }
  return 0;
}

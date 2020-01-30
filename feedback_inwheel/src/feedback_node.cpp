#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "fstream"
#include "tuple"
#include "thread"

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
    const int pin_hs[2]  = {7 , 16}; // {7, 16, 20, 21, 25}
    const int FREQ       = 10000;
    const int FORWARD    = 1;
    const int REVERSE    = -1;
    int u_r_in = 0;
    int u_l_in = 0;
    double v_r = 0.0;
    double v_l = 0.0;
    int r_dir  = 0;
    int l_dir  = 0;
  public:
    Serial(){
      pi = pigpio_start("localhost","8888");
      if(pi < 0){
        printf("GPIO ERROR");
      }
      for (int i = 0; i < 2; i++){
        set_mode(pi, pin_pwm[i], PI_OUTPUT);
        set_mode(pi, pin_dir[i], PI_OUTPUT);
      }
      for (int i = 0; i < 2; i++){
        set_mode(pi, pin_hs[i], PI_INPUT);
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
        r_dir = REVERSE;
      }else{
        gpio_write(pi, pin_dir[0], PI_HIGH); // タイヤの回転方向指定
        u_r_in = u_r;                       // 入力周波数指定
        r_dir = FORWARD;
      }
      if(u_l < 0){
        gpio_write(pi, pin_dir[1], PI_HIGH); // タイヤの回転方向指定
        u_l_in = abs(u_l);                  // 入力周波数指定
        l_dir = REVERSE;
      }else{
        gpio_write(pi, pin_dir[1], PI_LOW);  // タイヤの回転方向指定
        u_l_in = u_l;                       // 入力周波数指定
        l_dir = FORWARD;
      }

      hardware_PWM(pi, pin_pwm[0], FREQ, u_r_in);
      hardware_PWM(pi, pin_pwm[1], FREQ, u_l_in);
    };
    void enc_thread_r(const float r){
      ros::Time r_prev = ros::Time::now();
      ros::Time r_now;
      ros::Rate rate(100);
      double    r_dt = 0.0;
      int       r_level_prev = gpio_read(pi, pin_hs[0]);
      int       r_level_now  = r_level_prev;

      while(ros::ok()){
        r_level_now = gpio_read(pi, pin_hs[0]);
        if(r_level_now != r_level_prev && r_level_now == PI_LOW){
          r_now  = ros::Time::now();
          r_dt   = (r_now-r_prev).toSec();

          v_r = (2*M_PI*r)/(15*r_dt);

          r_level_prev = r_level_now;
          r_prev = r_now;
        }
        rate.sleep();
      }
    };
    void enc_thread_l(const float r){
      ros::Time l_prev = ros::Time::now();
      ros::Time l_now;
      ros::Rate rate(100);
      double    l_dt = 0.0;
      int       l_level_prev = gpio_read(pi, pin_hs[1]);
      int       l_level_now  = l_level_prev;

      while(ros::ok()){
        l_level_now = gpio_read(pi, pin_hs[1]);
        if(l_level_now != l_level_prev && l_level_now == PI_LOW){
          l_now = ros::Time::now();
          l_dt  = (l_now-l_prev).toSec();

          v_l = (2*M_PI*r)/(15*l_dt);

          l_level_prev = l_level_now;
          l_prev = l_now;
        }
        rate.sleep();
      }
    };
    void thread_start(const float r){
      thread thread_r( &Serial::enc_thread_r, this, r );
      thread thread_l( &Serial::enc_thread_l, this, r );

      thread_r.detach();
      thread_l.detach();
    };
    tuple<double, double> get_enc(){
      double v_r_dir = r_dir * v_r;
      double v_l_dir = l_dir * v_l;
      return {v_r_dir, v_l_dir};
    };
};

class Controller{
  private:
    /* ゲイン*/
    const float Kx  = 2;
    const float Ky  = 2;
    const float Kth = 2;
    /* 目標速度及び回転速度 */
    const float v_d  = 0.01;
    const float om_d = 0.01;
    /* カート本体のパラメータ */
    const float  r = 0.17/2; // タイヤ半径
    const float  T = 0.61;   // トレッド
    /* saturation */
    const int   MAX_DUTY = 1000000;
    const float MAX_V    = 1.5;
    const float MAX_OM   = M_PI;
    /* debugmode */
    const bool debug_flg = true;
    /* カート */
    Cartbot cart;
    /* GPIO */
    Serial ser;
    /* 速度入力 */
    double u_v  = 0.0;
    double u_om = 0.0;
    /* エンコーダによる速度 */
    double v_enc  = 0.0;
    double om_enc = 0.0;
    /* PWM入力 */
    int u_r  = 0;
    int u_l  = 0;
    /* モータ速度 */
    double v_r = 0.0;
    double v_l = 0.0;
    /* 位置姿勢 */
    double x  = 0.0;
    double y  = 0.0;
    double th = 0.0;
    /* 偏差 */
    double x_e  = 0.0;
    double y_e  = 0.0;
    double th_e = 0.0;
    /* csv用 */
    ofstream fs;
    ros::Time start   = ros::Time::now();
    ros::Time current = start;
  public:
    Controller(){
      ser.thread_start(r);
      if(debug_flg){
        fs.open("~/catkin_ws/src/feedback_inwheel/csv/"+to_string(std::time(nullptr))+".csv");
        fs << "t,x,y,th,v_enc,om_enc,u_v,u_om,u_r,u_l,v_r,v_l,x_e,y_e,th_e,Kx,Ky,Kth,v_d,om_d" << endl;
        output_statuses(true);
      }
    };
    ~Controller(){
      if(debug_flg){
        fs.close();
      }
    }
    void run(double x_d, double y_d, double th_d, double dt){
      auto status = cart.update(u_v, u_om, dt);
      x = get<0>(status);
      y  = get<1>(status);

      x_e  = (x_d-x)*cos(th) + (y_d-y)*sin(th);
      y_e  = (y_d-y)*cos(th) - (x_d-x)*sin(th);
      th_e = th_d-th;

      u_v  = v_d*cos(th_e) + Kx*x_e;
      u_om = om_d + Ky*y_e*v_d + Kth*sin(th_e);

      if(u_v > MAX_V){
        u_v = MAX_V;
      }else if(u_v < -MAX_V){
        u_v = -MAX_V;
      }
      if(u_om > MAX_OM){
        u_om = MAX_OM;
      }else if(u_om < -MAX_OM){
        u_om = -MAX_OM;
      }

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
      if(debug_flg){
        output_statuses();
      }
    };
    void get_u(){
      auto u = ser.get_enc();
      v_r = get<0>(u);
      v_l = get<1>(u);

      v_enc  = (v_r + v_l)/2;
      om_enc = (v_r - v_l)/T;
      u_v  = v_enc;
      u_om = om_enc;
    };
    void output_statuses(bool setup_flg = false){
      current = ros::Time::now();
      fs << (current-start).toSec() << ",";
      fs << x << "," << y << "," << th << ",";
      fs << v_enc << "," << om_enc<< ",";
      fs << u_v << "," << u_om << ",";
      fs << u_r << "," << u_l << ",";
      fs << v_r << "," << v_l << ",";
      fs << x_e << "," << y_e << "," << th_e;
      if(setup_flg){
        fs << "," << Kx << "," << Ky << "," << Kth << ",";
        fs << v_d << "," << om_d;
      }
      fs << endl;
    };
};

int main(int argc, char **argv){
  ros::init(argc, argv, "feedback_inwheel");
  ros::NodeHandle n;

  ros::Rate rate(20);

  ros::Time prev = ros::Time::now();;
  ros::Time now  = prev;

  double x_d  = 0;
  double y_d  = 2;
  double th_d = M_PI/2;
  double dt   = 0;

  Controller c;

  rate.sleep();

  while(ros::ok()){
    now = ros::Time::now();
    dt = (now-prev).toSec();

    c.run(x_d, y_d, th_d, dt);

    ros::spinOnce();
    rate.sleep();

    c.get_u();

    prev = now;
  }
  return 0;
}

#include "ros/ros.h"
#include "cmath"
#include "tuple"

using namespace std;

class Cartbot{
  private:
    double x;
    double y;
    double th;
    double v;
    double om;
  public:
    Cartbot(){
      x = 0;
      y = 0;
      th = M_PI/2;
      v = 0;
      om = 0;
    };

    tuple<double, double, double, double, double> update(double u_v, double u_om, double dt){
      x  = x + u_v * dt * cos(th+u_om*dt/2);
      y  = y + u_v * dt * sin(th+u_om*dt/2);
      th = th + u_om*dt;
      v  = u_v;
      om = u_om;

      return {x, y, th, v, om};
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

      x_e  =    (x_r-x)*cos(th) + (y_r-y)*sin(th);
      y_e  = -1*(x_r-x)*sin(th) + (y_r-y)*cos(th);
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


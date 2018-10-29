#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "pigpiod_if2.h"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define TIMEDIFF(now, prev, dt) (((now)-(prev))/(dt))

#define HALF 500000 //duty比
#define PHASE_DIFF 2*M_PI //位相遅れ[rad]
#define R 0.0036*M_PI/180.0
#define KP 0.3
#define KI 0.1
#define KD 0.05

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {21, 20};

static float d = 0.66/2; //タイヤ間距離[m]
static float r = 0.15/2; //タイヤ半径[m]

double l =0.0;     //対象点までの距離[m]
double theta = 0.0; //対象点までの角度[rad]

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  int count = scan->scan_time / scan->time_increment;
  double tmpl = 100.0;
  double tmptheta = 0.0;
  for(int i = 0; i < count; i++){
    double rad = scan->angle_min + scan->angle_increment * i;
    if(0<std::sin(rad)){
      if(scan->ranges[i] < tmpl){
        tmpl = scan->ranges[i];
        tmptheta = rad;
      }
    }
  }
  l= tmpl;
  theta = tmptheta;
}

void changeGPIO(int status){
  for (int i = 0; i < 2; i++){
    set_mode(pi, pwmpin[i], status);
    set_mode(pi, dirpin[i], status);
  }
}

void stopPulse(){
  for (int i = 0; i < 2; i++){
    hardware_PWM(pi, pwmpin[i], 0, 0);
    gpio_write(pi, dirpin[i], PI_LOW);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pigpio_test");
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Rate loop_rate(10);
  ros::Time prev;
  ros::Time now;
  ros::Duration duration;

  pi = pigpio_start("localhost","8888");
  changeGPIO(PI_OUTPUT);

// カート用変数
  double u_l   = 0.0; double u_r   = 0.0; // 左右モータへの入力周波数
  double v     = 0.0; double ohm   = 0.0; // 極座標での速度,角速度
  double dx_c  = 0.0; double dy_c  = 0.0; // xy座標での速度
  double d2x_c = 0.0; double d2y_c = 0.0; // xy座標での加速度
// 対象点P用変数
  double x_pprev  = 0.0; double y_pprev = 0.0;  // 離散時間 n-1 での位置
  double x_p      = 0.0; double y_p     = 0.0;  // 離散時間 n   での位置
  double dx_pprev = 0.0; double dy_pprev = 0.0; // 離散時間 n-1 での速度
  double dx_p     = 0.0; double dy_p    = 0.0;  // 離散時間 n   での速度
  double d2x_p    = 0.0; double d2y_p   = 0.0;  // 離散時間 n   での加速度
// 制御用変数
  double e_x     = 0.0; double e_y     = 0.0; // 位置偏差
  double e_xprev = 0.0; double e_yprev = 0.0; // 速度偏差
  double u_x     = 0.0; double u_y     = 0.0; // 偏差合計(一時変数)
  double tmpIx   = 0.0; double tmpIy   = 0.0; // 積分項
  double alpha   = 0.0;                       // 機体中心から対象点までのずれ角度
  double tmpSqrt = 0.0;                       // 計算量削減のための一時変数
  double dt      = 0.0;                       // 制御周期
  bool lost = false;                          // 対象点があるかないか
  bool setupFlg = false;                      // 点の初期設定フラグ
  int setupCount = 0;                         // 点の初期設定カウント

  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

// 対象点Pの初期設定
  while(!setupFlg){
    prev = ros::Time::now();
    ros::spinOnce();
    loop_rate.sleep();

    now = ros::Time::now();
    duration = now - prev;
    dt = duration.toSec();

    switch(setupCount){
      case 0:
        ROS_INFO_STREAM("Scaning Position ...");
        x_p = l*std::cos(theta-M_PI/2);
        y_p = l*std::sin(theta-M_PI/2);
        ROS_INFO("x0, y0 = %lf, %lf", x_p, y_p);
        if(x_p != 0.0 && y_p != 0.0)
          setupCount += 1;
        break;
      case 1:
        ROS_WARN_STREAM("Calculating Speed ...");
        x_pprev = x_p;
        y_pprev = y_p;
        x_p = l*std::cos(theta-M_PI/2);
        y_p = l*std::sin(theta-M_PI/2);
        dx_p = TIMEDIFF(x_p, x_pprev, dt);
        dy_p = TIMEDIFF(y_p, y_pprev, dt);
        ROS_INFO("dx0, dy0 = %lf, %lf", dx_p, dy_p);
        setupCount += 1;
        break;
      case 2:
        ROS_WARN_STREAM("Calculating Acceleration ...");
        x_pprev = x_p;
        y_pprev = y_p;
        x_p = l*std::cos(theta-M_PI/2);
        y_p = l*std::sin(theta-M_PI/2);
        dx_pprev = dx_p;
        dy_pprev = dy_p;
        dx_p = TIMEDIFF(x_p, x_pprev, dt);
        dy_p = TIMEDIFF(y_p, y_pprev, dt);
        d2x_p = TIMEDIFF(dx_p, dx_pprev, dt);
        d2y_p = TIMEDIFF(dy_p, dy_pprev, dt);
        ROS_INFO("d2x0, d2y0 = %lf, %lf", d2x_p, d2y_p);
        setupFlg = true;
        break;
    }
    ROS_INFO("P=(%2lf, %2lf), dP=(%2lf, %2lf), d2P=(%2lf, %2lf)", x_p, y_p, dx_p, dy_p, d2x_p, d2y_p);
  }

  while(!ros::ok()){ //上確認用、反転消す
    prev = ros::Time::now();

    loop_rate.sleep();
    ros::spinOnce();

    now = ros::Time::now();
    duration = now - prev;
    dt = duration.toSec();

    if(!lost){
      alpha = theta-M_PI;
      x_pprev = x_p;
      y_pprev = y_p;
      x_p = l*std::cos(theta-M_PI/2);
      y_p = l*std::sin(theta-M_PI/2);
      dx_pprev = dx_p;
      dy_pprev = dy_p;
      dx_p = TIMEDIFF(x_p, x_pprev, dt);
      dy_p = TIMEDIFF(y_p, y_pprev, dt);
      d2x_p = TIMEDIFF(dx_p, dx_pprev, dt);
      d2y_p = TIMEDIFF(dy_p, dy_pprev, dt);
      ROS_INFO("P=(%2lf, %2lf), dP=(%2lf, %2lf), d2P=(%2lf, %2lf)", x_p, y_p, dx_p, dy_p, d2x_p, d2y_p);

      e_xprev = e_x;
      e_yprev = e_y;
      e_x = x_p + dx_p - x_c;
      e_y = y_p + dy_p - y_c;

      if(e_x*e_x < 0.25)
        e_x = 0.0;
      if(y_p < 1.0)
        e_y = 0.0;

      tmpIx += e_x * dt;
      tmpIy += e_y * dt;

      tmpEx = KP*e_x + KI*tmpIx + KD*TIMEDIFF(e_x, e_xprev, dt);
      tmpEy = KP*e_y + KI*tmpIy + KD*TIMEDIFF(e_y, e_yprev, dt);

      tmpSqrt = std::sqrt(tmpEx*tmpEx+tmpEy*tmpEy)/(R*r);

      u_r = (1+2*d*std::sin(alpha)/l)*tmpSqrt;
      u_l = (1-2*d*std::sin(alpha)/l)*tmpSqrt;

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

      dx_c = v*std::cos(ohm*dt+M_PI/2);
      dy_c = v*std::sin(ohm*dt+M_PI/2);
    }
  }

  // 出力信号の停止
  stopPulse();

  // PINOUT -> PININ
  changeGPIO(PI_INPUT);

  // 終了
  pigpio_stop(pi);

  return 0;
}


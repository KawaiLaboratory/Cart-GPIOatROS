#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define TIMEDIFF(now, prev, dt) (((now)-(prev))/(dt))

#define HALF 500000 //duty比
#define PHASE_DIFF 2*M_PI //位相遅れ[rad]
#define R 0.0036*M_PI/180.0
#define KP 0.3
#define KI 0.0
#define KD 0.05

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {21, 20};

static double d = 0.66/2; //タイヤ間距離[m]
static double r = 0.15/2; //タイヤ半径[m]

double x_p = 0.0;
double y_p = 0.0;
bool lost = true;

void callback(const std_msgs::Float32MultiArray::ConstPtr& status){
  lost = (status->data[0] != 0.0);
  if(!lost){
    double x_t = status->data[1];
    double y_t = status->data[2];
    y_p = x_t;
    x_p = -y_t;
  }
  ROS_INFO("x:%f, y:%f", x_p, y_p);
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
  ros::Rate loop_rate(20);
  ros::Time prev;
  ros::Time now;
  ros::Duration duration;

  pi = pigpio_start("localhost","8888");
  changeGPIO(PI_OUTPUT);

// カート用変数
  double u_l   = 0.0; double u_r   = 0.0; // 左右モータへの入力周波数
  double v     = 0.0; double ohm   = 0.0; // 極座標での速度,角速度
  double x_c   = 0.0; double y_c   = 0.0;
  double dx_c  = 0.0; double dy_c  = 0.0; // xy座標での速度
  double phi   = M_PI/2;
// 対象点P用変数
  double x_pprev  = 0.0; double y_pprev  = 0.0;  // 離散時間 n-1 での位置
  double dx_pprev = 0.0; double dy_pprev = 0.0; // 離散時間 n-1 での速度
  double dx_p     = 0.0; double dy_p     = 0.0;  // 離散時間 n   での速度
// 制御用変数
  double e_x     = 0.0; double e_y     = 0.0; // 位置偏差
  double e_xprev = 0.0; double e_yprev = 0.0; // 速度偏差
  double x_e     = 0.0; double y_e     = 0.0; // 偏差合計(一時変数)
  double tmpIx   = 0.0; double tmpIy   = 0.0; // 積分項
  double alpha   = 0.0;                       // 機体中心から対象点までのずれ角度
  double l       = 0.0;
  double theta   = 0.0;
  double tmpSqrt = 0.0;                       // 計算量削減のための一時変数
  double dt      = 0.0;                       // 制御周期
  bool setupFlg = false;                      // 点の初期設定フラグ
  int setupCount = 0;                         // 点の初期設定カウント

  // sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
  sub = n.subscribe("/status", 1000, callback);

// 対象点Pの初期設定
  while(!setupFlg){
    x_pprev = x_p;
    y_pprev = y_p;
    prev = ros::Time::now();

    ros::spinOnce();
    loop_rate.sleep();

    now = ros::Time::now();
    duration = now - prev;
    dt = duration.toSec();

    switch(setupCount){
      case 0:
        ROS_INFO_STREAM("Scaning Position ...");
        if(x_p != 0.0 && y_p != 0.0)
          setupCount += 1;
        break;
      case 1:
        ROS_WARN_STREAM("Calculating Speed ...");
        dx_p = TIMEDIFF(x_p, x_pprev, dt);
        dy_p = TIMEDIFF(y_p, y_pprev, dt);
        ROS_INFO("dx(0), dy(0) = %lf, %lf", dx_p, dy_p);
        setupFlg = true;
        break;
    }
    ROS_INFO("P=(%2lf, %2lf), dP=(%2lf, %2lf)", x_p, y_p, dx_p, dy_p);
  }

  while(ros::ok()){
    x_pprev = x_p;
    y_pprev = y_p;
    prev = ros::Time::now();

    ros::spinOnce();
    loop_rate.sleep();

    now = ros::Time::now();
    duration = now - prev;
    dt = duration.toSec();

    dx_p = TIMEDIFF(x_p, x_pprev, dt);
    dy_p = TIMEDIFF(y_p, y_pprev, dt);
    ROS_INFO("P=(%2lf, %2lf), dP=(%2lf, %2lf)", x_p, y_p, dx_p, dy_p);


    if(lost){
      x_c += dx_c * dt;
      y_c += dy_c * dt;
      phi += ohm * dt;
    }else{
      x_c = 0.0;
      y_c = 0.0;
      phi = M_PI/2;
    }

    e_xprev = e_x;
    e_yprev = e_y;

    if((x_p-x_c)*(x_p-x_c)<0.25)  e_x = 0.0;
    //else                  e_x = (x_p - x_c)*(x_p - x_c)-0.25 + dx_p;
    else                          e_x = (x_p - x_c)*(x_p - x_c) - 0.25;
    if(y_p - y_c < 1)             e_y = 0.0;
    //else                  e_y = y_p - y_c - 1 + dy_p;;
    else                          e_y =  y_p - y_c - 1;

    ROS_INFO("%lf, %lf", e_x, e_y);
    alpha = std::atan2(y_p-y_c, x_p-x_c);
    l     = std::sqrt((x_p-x_c)*(x_p-x_c)+(y_p-y_c)*(y_p-y_c));

    tmpIx += e_x * dt;
    tmpIy += e_y * dt;

    x_e = KP*e_x + KI*tmpIx + KD*TIMEDIFF(e_x, e_xprev, dt);
    y_e = KP*e_y + KI*tmpIy + KD*TIMEDIFF(e_y, e_yprev, dt);

    ROS_INFO("x_e:%lf, y_e:%lf", x_e, y_e);
    tmpSqrt = std::sqrt(x_e*x_e+y_e*y_e)/(R*r);

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
    ROS_INFO("v:%lf", v);

    ROS_INFO("ur: %lf, ul: %lf", u_r, u_l);
    dx_c = v*std::cos(ohm*dt+M_PI/2);
    dy_c = v*std::sin(ohm*dt+M_PI/2);
  }

  // 出力信号の停止
  stopPulse();

  // PINOUT -> PININ
  changeGPIO(PI_INPUT);

  // 終了
  pigpio_stop(pi);

  return 0;
}

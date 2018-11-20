#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define TIMEDIFF(now, prev, dt) (((now)-(prev))/(dt))

#define HALF 500000 //duty比
#define PHASE_DIFF 2*M_PI //位相遅れ[rad]
#define R 0.0036*M_PI/180.0
#define KP 0.1
#define KI 0.01
#define KD 0.05

int pi;
extern int pi;

static int pwmpin[2] = {18, 19};
static int dirpin[2] = {20, 21};

static double d = 0.66/2; //タイヤ間距離[m]
static double r = 0.15/2; //タイヤ半径[m]

double x_p = 0.0;
double y_p = 0.0;
bool lost = true;

void callback(const std_msgs::Float32MultiArray::ConstPtr& status){
  lost = (status->data[0] != 0.0);
  if(!lost){
    x_p = -1*status->data[2];
    y_p = status->data[1];
  }
  //ROS_INFO("x:%f, y:%f", x_p, y_p);
}

void changeGPIO(int status){
  if(status == PI_INPUT){
    for (int i = 0; i < 2; i++){
      hardware_PWM(pi, pwmpin[i], 0, 0);
      gpio_write(pi, dirpin[i], PI_LOW);
    }
  }
  for (int i = 0; i < 2; i++){
    set_mode(pi, pwmpin[i], status);
    set_mode(pi, dirpin[i], status);
  }
}

double scaning(){
  ros::Rate loop_rate(1);
  ros::Time prev = ros::Time::now();

  ros::spinOnce();
  loop_rate.sleep();

  ros::Time now = ros::Time::now();
  return (now - prev).toSec();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pigpio_test");
  ros::NodeHandle n;
  ros::Subscriber sub;

  pi = pigpio_start("localhost","8888");
  changeGPIO(PI_OUTPUT);

// カート用変数
  double u_l   = 0.0; double u_r   = 0.0; // 左右モータへの入力周波数
  double v     = 0.0; double ohm   = 0.0; // 極座標での速度,角速度
  double x_c   = 0.0; double y_c   = 0.0;
  double dx_c  = 0.0; double dy_c  = 0.0; // xy座標での速度
  double phi   = M_PI/2;
// 制御用変数
  double e_x     = 0.0; double e_y     = 0.0; // 位置偏差
  double e_xprev = 0.0; double e_yprev = 0.0; // 速度偏差
  double x_e     = 0.0; double y_e     = 0.0; // 偏差合計(一時変数)
  double tmpIx   = 0.0; double tmpIy   = 0.0; // 積分項
  double dt      = 0.0;                       // 制御周期
  double alpha   = 0.0; double l       = 0.0; // 角度誤差, 距離誤差
  bool setupFlg = false;                      // 点の初期設定フラグ
  int setupCount = 0;                         // 点の初期設定カウント

  sub = n.subscribe("/status", 1000, callback);

  while(!setupFlg){
    dt = scaning();

    switch(setupCount){
      case 0:
        ROS_INFO_STREAM("Scaning Position ...");
        if(x_p != 0.0 && y_p != 0.0)
          setupFlg = true;
        break;
    }
  }

  while(ros::ok()){
    dt = scaning();

    if(lost){
      x_c += dx_c * dt;
      y_c += dy_c * dt;
      phi += ohm * dt;
    }else{
      x_c = 0.0;
      y_c = 0.0;
      phi = M_PI/2;
    }

    e_xprev   = e_x;
    e_yprev   = e_y;

    if(std::abs(x_p-x_c)<0.5)      e_x   = 0.0;
    else                           e_x   = x_p - x_c;
    if(0 < y_p-y_c && y_p-y_c < 1) e_y   = 0.0;
    else                           e_y   = y_p - y_c;
    if(e_x == 0.0 && e_y == 0.0)   alpha = 0.0;
    else                           alpha = std::atan2(e_y, e_x)-phi;

    tmpIx   += e_x*dt;
    tmpIy   += e_y*dt;

    x_e   = KP*e_x   + KI*tmpIx   + KD*TIMEDIFF(e_x, e_xprev, dt);
    y_e   = KP*e_y   + KI*tmpIy   + KD*TIMEDIFF(e_y, e_yprev, dt);

    u_r = 1/(R*r)*std::sqrt(x_e*x_e+y_e*y_e)*(1+2*d*std::sin(alpha));
    u_l = 1/(R*r)*std::sqrt(x_e*x_e+y_e*y_e)*(1-2*d*std::sin(alpha));

    ROS_INFO("r:%lf, l:%lf", u_r, u_l);

    if(u_r < 0){
      gpio_write(pi, dirpin[0], PI_LOW);
      u_r = std::fabs(u_r);
    }else{
      gpio_write(pi, dirpin[0], PI_HIGH);
    }

    if(u_l < 0){
      gpio_write(pi, dirpin[1], PI_HIGH);
      u_l = std::fabs(u_l);
    }else{
      gpio_write(pi, dirpin[1], PI_LOW);
    }

    hardware_PWM(pi, pwmpin[0], (int)u_l, HALF);
    hardware_PWM(pi, pwmpin[1], (int)u_r, HALF);

    v   = R*r/2*(u_r + u_l);
    ohm = R*r/(2*d)*(u_r - u_l);

    dx_c = v*std::cos(ohm*dt+M_PI/2);
    dy_c = v*std::sin(ohm*dt+M_PI/2);
  }
  // PINOUT -> PININ
  changeGPIO(PI_INPUT);

  // 終了
  pigpio_stop(pi);

  return 0;
}

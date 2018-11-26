#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

/* --- 時間微分 --- */
#define TIMEDIFF(now, prev, dt) (((now)-(prev))/(dt))

/* --- 各種定数 --- */
#define HALF 500000         // 周波数のデューティ比[%]
#define R 0.0036*M_PI/180.0 // モータの分解能[rad]
#define r 0.15/2            // タイヤ半径[m]
#define d 0.66/2            // 期待幅/2[m]
#define KP 0.1              // 比例ゲイン
#define KI 0.0             // 積分ゲイン #基本的に0
#define KD 0.05             // 微分ゲイン

/* --- 各種グローバル変数 --- */
int pi;                           // GPIO用
extern int pi;                    // 同上
static int pwmpin[2] = {18, 19};  // 速度入力用ピン
static int dirpin[2] = {20, 21};  // 回転方向用ピン
double x_p = 0.0;                 // 目標点のx座標
double y_p = 0.0;                 // 目標点のy座標
bool lost = true;                 // 対象の認識T/F
bool driving_flg = false;

void PointCallback(const std_msgs::Float32MultiArray::ConstPtr& status){
  lost = (status->data[0] != 0.0);
  if(!lost){
    x_p = -1*status->data[2];
    y_p = status->data[1];
  }
}

void FlagCallback(const std_msgs::Bool::ConstPtr& flag){
  driving_flg = flag->data;
  ROS_INFO("%d", driving_flg);
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
  ros::Subscriber p_sub; // LRFのサブスクライバ
  ros::Subscriber f_sub; // フラグ読み取りのサブスクライバ

  pi = pigpio_start("localhost","8888");
  changeGPIO(PI_OUTPUT);

// カート用変数
  double u_l   = 0.0; double u_r   = 0.0; // 左右モータへの入力周波数
  double v     = 0.0; double ohm   = 0.0; // 極座標での速度,角速度
  double x_c   = 0.0; double y_c   = 0.0;
  double phi   = M_PI/2;
  double dx_c  = 0.0; double dy_c  = 0.0; // xy座標での速度
// 制御用変数
  double e_x     = 0.0; double e_y     = 0.0; // 位置偏差
  double e_xprev = 0.0; double e_yprev = 0.0; // 速度偏差
  double u_x     = 0.0; double u_y     = 0.0; // 偏差合計(一時変数)
  double tmpIx   = 0.0; double tmpIy   = 0.0; // 積分項
  double dt      = 0.0;                       // 制御周期
  double alpha   = 0.0; double l       = 0.0; // 角度誤差, 距離誤差
  bool setupFlg = false;                      // 点の初期設定フラグ

  p_sub = n.subscribe("/status", 1000, PointCallback);
  f_sub = n.subscribe("/flag", 1000, FlagCallback);

  while(!setupFlg){
    dt = scaning();

    ROS_INFO_STREAM("Scaning Position ...");
    setupFlg = (x_p != 0.0 && y_p != 0.0)? true : false;
  }

  while(ros::ok()){
    if(driving_flg){
      /*====入力部分====*/
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

      dx_c = v*std::cos(phi);
      dy_c = v*std::sin(phi);

      if(lost){
        x_c += dx_c * dt;
        y_c += dy_c * dt;
        phi += ohm * dt;
      }else{
        x_c = 0.0;
        y_c = 0.0;
        phi = M_PI/2;
      }

      /*====比較部分====*/
      dt = scaning();

      e_xprev = e_x;
      e_yprev = e_y;

      e_x = (std::abs(x_p-x_c)<0.5)? 0.0 : x_p - x_c;
      e_y = (0 < y_p-y_c && y_p-y_c < 1)? 0.0 : y_p - y_c;

      /*====フィードバック部分====*/
      tmpIx += e_x*dt;
      tmpIy += e_y*dt;

      alpha = std::atan2(e_y, e_x)-phi;

      u_x = KP*e_x + KI*tmpIx + KD*TIMEDIFF(e_x, e_xprev, dt);
      u_y = KP*e_y + KI*tmpIy + KD*TIMEDIFF(e_y, e_yprev, dt);

      u_r = 1/(R*r)*std::sqrt(u_x*u_x+u_y*u_y)*(1+2*d*std::sin(alpha));
      u_l = 1/(R*r)*std::sqrt(u_x*u_x+u_y*u_y)*(1-2*d*std::sin(alpha));
    }else{
      stopPulse();
    }
  }

  // PINOUT -> PININ
  stopPulse();
  changeGPIO(PI_INPUT);

  // 終了
  pigpio_stop(pi);

  return 0;
}

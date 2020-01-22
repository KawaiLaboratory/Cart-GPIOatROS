#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "fstream"
#include "ctime"

/* --- 各種定数 --- */
#define HALF 500000         // 周波数のデューティ比[%]
#define R 0.0036*M_PI/180.0 // モータの分解能[rad]
#define r 0.15/2            // タイヤ半径[m]
#define d 0.66/2            // 期待幅/2[m]
#define KP 0.4              // 比例ゲイン
#define KI 0.0              // 積分ゲイン(基本的に0)
#define KD 0.04             // 微分ゲイン

/* --- 各種グローバル変数 --- */
int pi;                           // GPIO用
extern int pi;                    // 同上
static int pwmpin[2] = {18, 19};  // 速度入力用ピン
static int dirpin[2] = {20, 21};  // 回転方向用ピン
double x_p = 0.0;                 // 目標点のx座標
double y_p = 0.0;                 // 目標点のy座標
bool lost = true;                 // 対象の認識T/F
bool driving_flg = false;         // 運転開始フラグ

/*--- 目標点コールバック関数 ---*/
void PointCallback(const std_msgs::Float32MultiArray::ConstPtr& status){
  lost = (status->data[0] != 0.0);
  if(!lost){
    x_p = -1*status->data[2];   // LRF設置の角度補正
    y_p = status->data[1];      // 同上
  }
}

/*--- 運転準備完了フラグコールバック関数 ---*/
void FlagCallback(const std_msgs::Bool::ConstPtr& flag){
  driving_flg = flag->data;     // 運転開始かどうか購読
}

/*--- GPIOの開始及び終了用関数 ---*/
void changeGPIO(int status){
  for (int i = 0; i < 2; i++){
    set_mode(pi, pwmpin[i], status);
    set_mode(pi, dirpin[i], status);
  }
}

/*--- PWM信号の停止用関数 ---*/
void stopPulse(){
  for (int i = 0; i < 2; i++){
    hardware_PWM(pi, pwmpin[i], 0, 0);
    gpio_write(pi, dirpin[i], PI_LOW);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pigpio_test");
  ros::NodeHandle n;
  ros::Subscriber p_sub; // LRFのサブスクライバ
  ros::Subscriber f_sub; // フラグ読み取りのサブスクライバ

  ros::Rate loop_rate(20);

  ros::Time prev;
  ros::Time now;

  pi = pigpio_start("localhost","8888");

  /*--- カート用変数 ---*/
  double u_l   = 0.0; double u_r   = 0.0; // 左右モータへの計算上の入力周波数
  double ul4in = 0.0; double ur4in = 0.0; // 左右モータへの実際の入力周波数(abs(u_hoge))
  double v     = 0.0; double ohm   = 0.0; // 極座標での速度,角速度
  double x_c   = 0.0; double y_c   = 0.0; // カートの位置
  double phi   = M_PI/2;                  // カートの姿勢
  double dx_c  = 0.0; double dy_c  = 0.0; // xy座標での速度
  /*--- 対象点用変数 ---*/
  double x_pprev = 0.0; double y_pprev = 0.0; // 目標点の前位置
  /*--- 制御用変数 ---*/
  double e_x     = 0.0; double e_y     = 0.0; // 位置偏差
  double u_x     = 0.0; double u_y     = 0.0; // PD制御による入力
  double tmpIx   = 0.0; double tmpIy   = 0.0; // 積分項用一時変数
  double dt      = 0.0; double time    = 0.0; // 制御周期, 待機時間
  double alpha   = 0.0; double l       = 1.0; // 角度誤差, 参照距離
  bool setupFlg = false;                      // 点の初期設定フラグ

  /*--- メッセージ購読 ---*/
  p_sub = n.subscribe("/status", 1000, PointCallback);    // 目標点座標及び検出TFの購読
  f_sub = n.subscribe("/driving_flag", 5, FlagCallback);  // 運転開始フラグの購読

  /*--- csv吐き出し用準備(実使用時は消す) ---*/
  std::ofstream fs("~/catkin_ws/src/pigpio_test/"+std::to_string(std::time(nullptr))+".csv");  // CSVファイル生成
  double time4csv = 0.0;                                        // 記録用時間

  /*--- 人物の初期位置の検出 ---*/
  while(!setupFlg){
    ros::spinOnce();
    loop_rate.sleep();

    ROS_INFO_STREAM("Scaning Position ...");
    setupFlg = (x_p != 0.0 && y_p != 0.0)? true : false;
  }

  /*--- csv書き込み(実使用時は消す) ---*/
  fs << "time,xp,xc,yp,yc,phi,ur,ul,lost,kp,ki,kd" << std::endl;
  fs << time4csv << "," << x_p << "," << x_c << "," << y_p << "," << y_c << "," << phi << ",";
  fs << u_r << "," << u_l << ","<< lost  << "," << KP << "," << KI << "," << KD << std::endl;

  prev = ros::Time::now();
  rate.sleep();
  
  while(ros::ok()){
    now = ros::Time::now();
    if(driving_flg){
      time = 0.0;
      /*=== 入力部分 ===*/
      changeGPIO(PI_OUTPUT);  // モータのGPIOピンを出力状態に変更

      /*--- 入力用に符号変更処理 ---*/
      if(u_r < 0){
        gpio_write(pi, dirpin[0], PI_LOW);  // タイヤの回転方向指定
        ur4in = std::fabs(u_r);             // 入力周波数指定
      }else{
        gpio_write(pi, dirpin[0], PI_HIGH); // タイヤの回転方向指定
        ur4in = u_r;                        // 入力周波数指定
      }
      if(u_l < 0){
        gpio_write(pi, dirpin[1], PI_HIGH); // タイヤの回転方向指定
        ul4in = std::fabs(u_l);             // 入力周波数指定
      }else{
        gpio_write(pi, dirpin[1], PI_LOW);  // タイヤの回転方向指定
        ul4in = u_l;                        // 入力周波数指定
      }

      /*--- PWM生成 ---*/
      hardware_PWM(pi, pwmpin[0], (int)ul4in, HALF);
      hardware_PWM(pi, pwmpin[1], (int)ur4in, HALF);

      /*--- カートの速度・加速度計算 ---*/
      v   = R*r/2*(u_r + u_l);
      ohm = R*r/(2*d)*(u_r - u_l);

      /*--- カートのxy座標での速度計算 ---*/
      dx_c = v*std::cos(phi);
      dy_c = v*std::sin(phi);

      /*--- 検出の可否で位置姿勢の場合分け ---*/
      if(lost){           // 非検出の場合
        x_c += dx_c * dt;
        y_c += dy_c * dt;
        phi += ohm * dt;
      }else{              // 検出できている場合
        x_c = 0.0;
        y_c = 0.0;
        phi = M_PI/2;
      }

      /*====比較部分====*/
      dt = (now-prev).toSec();   // 制御周期

      e_x = (std::abs(x_p-x_c)<0.5)?      0.0 : x_p - x_c;  // x座標の偏差
      e_y = (0 < y_p-y_c && y_p-y_c < 1)? 0.0 : y_p - y_c;  // y座標の偏差
      alpha = std::atan2(y_p - y_c, x_p - x_c)-phi;         // 角度偏差

      /*====フィードバック部分====*/
      tmpIx += e_x*dt;  // xの積分項
      tmpIy += e_y*dt;  // yの積分項

      u_x = KP*e_x - KD*dx_c; // PD制御による入力
      u_y = KP*e_y - KD*dy_c; // PD制御による入力

      u_r = 1/(R*r)*std::sqrt(u_x*u_x+u_y*u_y)*(1+2*d*std::sin(alpha)/l);   // PurePursuitアルゴリズムを用いたモータへの入力周波数
      u_l = 1/(R*r)*std::sqrt(u_x*u_x+u_y*u_y)*(1-2*d*std::sin(alpha)/l);   // PurePursuitアルゴリズムを用いたモータへの入力周波数

      x_pprev = x_p;  // 前位置の更新
      y_pprev = y_p;  // 前位置の更新

      /*--- csv書き込み用(実使用時は消す) ---*/
      time4csv += dt;
      fs << time4csv << "," << x_p << "," << x_c << "," << y_p << "," << y_c << "," << phi << ",";
      fs << u_r << "," << u_l << ","<< lost << std::endl;
    }else{
      stopPulse();          // PWMの停止
      changeGPIO(PI_INPUT); // GPIOを入力状態に変更

      dt = (now-prev).toSec();
      time += dt;    // 待機時間の加算

      /*--- 停止処理 ---*/
      if (time > 30.0){
        fs.close();
        ROS_INFO("system is shutdown!");
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();

    prev = now;
  }

  /*--- 信号出力の停止 ---*/
  stopPulse();
  changeGPIO(PI_INPUT);

  /*--- PIGPIOの終了 ---*/
  pigpio_stop(pi);

  return 0;
}

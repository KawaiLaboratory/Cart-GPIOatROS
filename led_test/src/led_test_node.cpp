#include "ros/ros.h"
#include "pigpiod_if2.h"
#include "math.h"
#include "std_msgs/Bool.h"

/* GPIOピン番号の宣言 */
#define SHUTDOWN_PIN 17
#define CLUTCH_PIN   27
#define START_PIN    22
#define SETUP_LED    26
#define DRIVING_LED  12

/* PIGPIOとの接続用 */
int pi;
extern int pi;

int main(int argc, char **argv){
  /*--- ROS用宣言 ---*/
  ros::init(argc, argv, "led_test");
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Rate loop_rate(20);

  /*--- 運転フラグ送信用宣言 ---*/
  pub = n.advertise<std_msgs::Bool>("driving_flag", 1000);
  std_msgs::Bool flag;

  /*--- PIGPIO制御 ---*/
  pi = pigpio_start("localhost", "8888");
  set_mode(pi, SHUTDOWN_PIN, PI_INPUT);   // 緊急停止スイッチ読み取り用
  set_mode(pi, CLUTCH_PIN, PI_INPUT);     // 電磁クラッチスイッチ読み取り用
  set_mode(pi, START_PIN, PI_INPUT);      // スタートスイッチ読み取り用
  set_mode(pi, SETUP_LED, PI_OUTPUT);     // 緑色LED出力用
  set_mode(pi, DRIVING_LED, PI_OUTPUT);   // 赤色LED出力用

  /*--- 処理用フラグ宣言  ---*/
  bool ClutchFlg   = false;               // クラッチのONOFF
  bool ShutdownFlg = false;               // 緊急停止スイッチのONOFF
  bool InitFlg     = false;               // 走行準備完了フラグ
  bool DrivingFlg  = false;               // 運転中フラグ

  /*--- 処理簡略化用変数宣言 ---*/
  int clutch_status = 0;
  int shutdown_status = 0;

  while(ros::ok()){
    /*--- クラッチと緊急停止スイッチの状態読み取り ---*/
    clutch_status   = gpio_read(pi, CLUTCH_PIN);
    shutdown_status = gpio_read(pi, SHUTDOWN_PIN);

    /*--- フラグ状態の切り替え処理 ---*/
    ShutdownFlg = (shutdown_status == PI_LOW)? true : false;  // 緊急停止スイッチのフラグ切り替え
    ClutchFlg   = (clutch_status   == PI_LOW)? true : false;  // クラッチのフラグ切り替え
    InitFlg     = (ClutchFlg && ShutdownFlg) ? true : false;  // 走行が可能か切り替え

    gpio_write(pi, SETUP_LED, InitFlg);                       // 緑色LEDの点灯消灯

    /*--- 運転状態の切り替え ---*/
    if(InitFlg){
      if(wait_for_edge(pi, START_PIN, FALLING_EDGE, 0.05))    // スタートスイッチの割り込み待機
        DrivingFlg = !DrivingFlg;                             // 運転開始フラグの反転
    }else{
      DrivingFlg = false;                                     // 運転開始フラグをへし折る
    }

    flag.data = DrivingFlg;                                   // 運転開始フラグの配信準備
    pub.publish(flag);                                        // 運転開始フラグの配信準備

    gpio_write(pi,DRIVING_LED, DrivingFlg);                   // 赤色LEDの点灯消灯

    /*--- ループ ---*/
    ros::spinOnce();  
    loop_rate.sleep();
  }
  gpio_write(pi, SETUP_LED, PI_LOW);
  set_mode(pi, SETUP_LED, PI_INPUT);
  gpio_write(pi, DRIVING_LED, PI_LOW);
  set_mode(pi, DRIVING_LED, PI_INPUT);
  pigpio_stop(pi);
}

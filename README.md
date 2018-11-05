# Cart-GPIOatROS
RaspberryPi内にあるROSのsrcのバックアップ

以下バージョン情報 そのうち追記
## ROS version
kinetic

## RaspberryPi OS version
raspbian

## About program
大半はRaspberryPiのROS導入で作られたやつ、消していいかわからないので残してある。
作ったやつは以下の4つ(2018-11-5時点)
- pigpio_test
  - c++
  - カート制御のメインプログラム。もっといい名前にしておけばよかった。
- udp-*
  - c++
  - 2台のRaspberryPiで通信するために作ってみたPublisherとSubscriber。ROSの通信で良いとなったので没プログラム。
- gui-test
  - Python
  - GUIでいろいろと情報を表示したいと思って作ったプログラム。余裕があればやる。
  

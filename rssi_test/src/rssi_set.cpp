#include <iostream>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <numeric>
#include <unistd.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;

double hci0_rssi = 0;
double hci1_rssi = 0;
double hci2_rssi = 0;

void HCI0Callback(const std_msgs::Float32::ConstPtr& msg){
  hci0_rssi = msg->data;
}

void HCI1Callback(const std_msgs::Float32::ConstPtr& msg){
  hci1_rssi = msg->data;
}

void HCI2Callback(const std_msgs::Float32::ConstPtr& msg){
  hci2_rssi = msg->data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rssi");
    ros::NodeHandle n;
    ros::Rate rate(10);
    ros::Subscriber sub0; // HCI0
    ros::Subscriber sub1; // HCI1
    ros::Subscriber sub2; // HCI2

    sub0 = n.subscribe("/hci0_ave", 5, HCI0Callback);
    sub1 = n.subscribe("/hci1_ave", 5, HCI1Callback);
    sub2 = n.subscribe("/hci2_ave", 5, HCI2Callback);

    ofstream log;
    log.open("/home/pi/catkin_ws/src/rssi_test/csvs/rssi.csv", ios::trunc);
    log << "hci0, hci1, hci2" << endl;

    while(ros::ok()){
        log << hci0_rssi << ",";
        log << hci1_rssi << ",";
        log << hci2_rssi;
        log << endl;

        ros::spinOnce();
        rate.sleep();
    }
    log.close();
    system("sudo kill `pidof hcidump`");

    system("sudo hcitool -i hci0 cmd 08 000c 00 01 > /dev/null");
    system("sudo hcitool -i hci1 cmd 08 000c 00 01 > /dev/null");
    system("sudo hcitool -i hci2 cmd 08 000c 00 01 > /dev/null");

    system("sudo kill `pidof hcitool` > /dev/null");
}

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <numeric>
#include <thread>
#include <unistd.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <ros/ros.h>

using namespace std;

const static char mac[18] = "D0:01:00:3E:64:4D";    // Beacon's Mac address
const static int  width   = 15;                     // filter's max width

double hci0_rssi = 0;
double hci1_rssi = 0;
double hci2_rssi = 0;

void BLE_hci0(){
    ros::Time start = ros::Time::now();
    ros::Time now;

    char s[256];                                        // Bluetooth's Data
    char c[256];                                        // Command
    int rssi = 0;                                       // RSSI
    int num = 0;                                        // SAMPLE's count
    deque<int> samples;                                 // SAMPLE
    FILE *fp;                                           // hcidump Command
    bool flg = false;

    // recording
    ofstream log;
    log.open("/home/pi/catkin_ws/src/rssi_test/csvs/hci0.csv", ios::trunc);
    log << "time, rssi" << endl;

    system("sudo hcitool -i hci0 lescan --pa --du > /dev/null &");

    while(ros::ok()){
        fp=popen("sudo hcidump -i hci0","r");
        flg = false;

        while(!flg && ros::ok()){
            fgets(s, 256, fp);
            if(strncmp(&s[4], "LE Advertising Report", 21) == 0){
                for(int i = 0; i < 6; i++){
                    fgets(s, 256, fp);
                    if(i == 1){
                        if(strncmp(&s[13], mac, 17) == 0) flg = true;
                    }
                }
            }
        }

        rssi = atoi(&s[12]);
        now = ros::Time::now();

        // recording
        log << (start - now).toSec(); << "," << rssi << endl;

        if(samples.size() > width){
            samples.pop_front();
        }
        samples.push_back(rssi);

        hci0_rssi = accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    }
    pclose(fp);
    log.close();
}

void BLE_hci1(){
    ros::Time start = ros::Time::now();
    ros::Time now;

    char s[256];                                        // Bluetooth's Data
    char c[256];                                        // Command
    int rssi = 0;                                       // RSSI
    int num = 0;                                        // SAMPLE's count
    deque<int> samples;                                 // SAMPLE
    FILE *fp;                                           // hcidump Command
    bool flg = false;

    // recording
    ofstream log;
    log.open("/home/pi/catkin_ws/src/rssi_test/csvs/hci1.csv", ios::trunc);
    log << "time, rssi" << endl;

    system("sudo hcitool -i hci1 lescan --pa --du > /dev/null &");

    while(ros::ok()){
        fp=popen("sudo hcidump -i hci1","r");
        flg = false;

        while(!flg && ros::ok()){
            fgets(s, 256, fp);
            if(strncmp(&s[4], "LE Advertising Report", 21) == 0){
                for(int i = 0; i < 6; i++){
                    fgets(s, 256, fp);
                    if(i == 1){
                        if(strncmp(&s[13], mac, 17) == 0) flg = true;
                    }
                }
            }
        }

        rssi = atoi(&s[12]);
        now = ros::Time::now();

        // recording
        log << (start - now).toSec(); << "," << rssi << endl;

        if(samples.size() > width){
            samples.pop_front();
        }
        samples.push_back(rssi);

        hci1_rssi = accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    }
    pclose(fp);
    log.close();
}

void BLE_hci2(){
    ros::Time start = ros::Time::now();
    ros::Time now;

    char s[256];                                        // Bluetooth's Data
    char c[256];                                        // Command
    int rssi = 0;                                       // RSSI
    int num = 0;                                        // SAMPLE's count
    deque<int> samples;                                 // SAMPLE
    FILE *fp;                                           // hcidump Command
    bool flg = false;

    // recording
    ofstream log;
    log.open("/home/pi/catkin_ws/src/rssi_test/csvs/hci2.csv", ios::trunc);
    log << "time, rssi" << endl;

    system("sudo hcitool -i hci2 lescan --pa --du > /dev/null &");

    while(ros::ok()){
        fp=popen("sudo hcidump -i hci2","r");
        flg = false;

        while(!flg && ros::ok()){
            fgets(s, 256, fp);
            if(strncmp(&s[4], "LE Advertising Report", 21) == 0){
                for(int i = 0; i < 6; i++){
                    fgets(s, 256, fp);
                    if(i == 1){
                        if(strncmp(&s[13], mac, 17) == 0) flg = true;
                    }
                }
            }
        }

        rssi = atoi(&s[12]);
        now = ros::Time::now();

        // recording
        log << (start - now).toSec(); << "," << rssi << endl;

        if(samples.size() > width){
            samples.pop_front();
        }
        samples.push_back(rssi);

        hci2_rssi = accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    }
    pclose(fp);
    log.close();
}


int main(int argc, char **argv){
    ros::init(argc, argv, "rssi_test");
    ros::NodeHandle n;

    thread hci0(BLE_hci0);
    thread hci1(BLE_hci1);
    thread hci2(BLE_hci2);

    ofstream log;
    int count = 0;
    log.open("/home/pi/catkin_ws/src/rssi_test/csvs/rssi.csv", ios::trunc);
    log << "count, hci0, hci1, hci2" << endl;

    hci0.detach();
    hci1.detach();
    hci2.detach();

    while(count < 60){
        sleep(1);
        log << count     << ",";
        log << hci0_rssi << ",";
        log << hci1_rssi << ",";
        log << hci2_rssi;
        log << endl;


        cout << hci0_rssi << ",";
        cout << hci1_rssi << ",";
        cout << hci2_rssi << endl;

        count++;
    }

    log.close();
    system("sudo hcitool -i hci0 cmd 08 000c 00 01 > /dev/null");
    system("sudo hcitool -i hci1 cmd 08 000c 00 01 > /dev/null");
    system("sudo hcitool -i hci2 cmd 08 000c 00 01 > /dev/null");
    system("sudo kill `pidof hcitool` > /dev/null");
    return 0;
}

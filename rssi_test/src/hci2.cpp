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

double diff_t(time_t start, time_t end){
    return (double)(end - start);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "hci2");
    ros::NodeHandle n;
    ros::Publisher pub;
    pub = n.advertise<std_msgs::Float32>("hci2_ave", 1000);
    std_msgs::Float32 msg;
    ros::Rate rate(10);

    char s[256];                                        // Bluetooth's Data
    char c[256];                                        // Command
    int rssi = 0;                                       // RSSI
    int num = 0;                                        // SAMPLE's count
    deque<int> samples;                                 // SAMPLE
    FILE *fp;                                           // hcidump Command
    const static char mac[18] = "D0:01:00:3E:64:4D";    // Beacon's Mac address
    const static int  width   = 15;                     // filter's max width

    bool flg = false;

    // recording
    time_t t_zero = time(NULL);
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

        // recording
        log << diff_t(t_zero, time(NULL)) << "," << rssi << endl;

        if(samples.size() > width){
            samples.pop_front();
        }
        samples.push_back(rssi);

        msg.data = accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
        pub.publish(msg);

        ros::spinOnce();
    }
    log.close();
    pclose(fp);
}

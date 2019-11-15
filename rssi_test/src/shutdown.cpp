#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "rssi_shutdown");
    ros::NodeHandle n;

	while(ros::ok()){}

    system("sudo kill `pidof hcidump`");

	system("sudo hcitool -i hci0 cmd 08 000c 00 01 > /dev/null");
    system("sudo hcitool -i hci1 cmd 08 000c 00 01 > /dev/null");
    system("sudo hcitool -i hci2 cmd 08 000c 00 01 > /dev/null");

	system("sudo kill `pidof hcitool` > /dev/null");
}
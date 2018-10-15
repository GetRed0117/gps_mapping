#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher pubControlInfo = n.advertise<std_msgs::String>("controlInfo", 1000);

    std::string line;

    while (getline(std::cin, line) && ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss(line);
        msg.data = ss.str();

        ROS_INFO("%s", "OK");

        pubControlInfo.publish(msg);

        
    }

    return 0;
}
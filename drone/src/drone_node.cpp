#include <drone.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "px4_general");
    ros::NodeHandle nodehandle;

    ROS_INFO("PX4 General Start");
    Drone drone(nodehandle);

    return 0;
}
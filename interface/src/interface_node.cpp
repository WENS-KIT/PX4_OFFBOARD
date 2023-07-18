#include "../include/interface.h"
// #include "interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interface_node");
    ros::NodeHandle nh;

    Interface interface(nh);

    return 0;
}


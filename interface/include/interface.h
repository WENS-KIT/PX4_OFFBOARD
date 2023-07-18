#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Header.h>

#include <map>
#include <sstream>

class Interface{
    private:
        ros::NodeHandle nh;

        ros::Publisher pub_gcsRequest;
        ros::Subscriber sub_droneRespond;

        ros::Subscriber sub_state;
        ros::Subscriber sub_localPose;
        ros::Subscriber sub_globalPose;
        ros::Subscriber sub_setPose;

        ros::Timer timer_display;

        double current_yaw = 0; //Radian
        double set_yaw = 0;

        
        geometry_msgs::PoseStamped current_localPose;
        sensor_msgs::NavSatFix current_globalPose;
        mavros_msgs::State current_state;

        geometry_msgs::PoseStamped set_pose;
        
        ros::Rate rate = ros::Rate(10.0);

        std_msgs::Header respond;
        std_msgs::Header request;
        double rtt_t = -1;

        std::map<int, std::string> system_status_map;
        std::array<std::string, 16> manual;
        std::array<std::string, 18> info;

        void PrintData();
        void PrintManual();

        void CallbackState(const mavros_msgs::State::ConstPtr& state_msg);
        void CallbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        void CallbackGlobalPose(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
        void CallbackSetPose(const geometry_msgs::PoseStamped::ConstPtr& setPose_msg);
        
        void CallbackDroneRespond(const std_msgs::Header::ConstPtr& drone_msg);
        void CallbackDisplay(const ros::TimerEvent& event);

    public:
        explicit Interface(const ros::NodeHandle& _nodeHandle);
        ~Interface();
        void _Teleop();
        void _Shutdown();
};
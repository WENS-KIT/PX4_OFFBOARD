#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <map>

class Drone{
    private:
        ros::NodeHandle nh;
    
        ros::Subscriber sub_state;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_gcsRequest; // get message from GCS

        ros::Publisher pub_setpoint_local;

        ros::ServiceClient client_arm;
        ros::ServiceClient client_mode;

        ros::Publisher pub_droneRespond;

        geometry_msgs::PoseStamped set_localPose;
        geometry_msgs::PoseStamped current_pose;

        double current_yaw = 0; //Radian

        mavros_msgs::State current_state;
        mavros_msgs::State prev_state;

        ros::Rate rate = ros::Rate(10.0);

        std::map<int, std::function<void()>> teleop_map;
        
        bool is_takeoff = false;
        bool enable_teleop = true;
        double takeoff_height = 2.0;

        std_msgs::Header request;
        std_msgs::Header respond;

        bool ReadParam();
        void TeleopMapping(std::map<int, std::function<void()>>&);

        bool CheckConnection();
        bool CheckArm(bool requested_arm);
        bool CheckFlying();
        bool CheckMode(std::string mode);

        void Arm();
        void ChangeMode(std::string mode);
        void Land();
        void Hover();
        
        void SetCurrentPose();
        void SetCurrentYaw();
        void MoveLinear(double pose_x, double pose_y, double pose_z);
        void MoveAngular(double degree);


        void CallbackState(const mavros_msgs::State::ConstPtr& state_msg);
        void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

        void CallbackGcsRequest(const std_msgs::Header::ConstPtr& gcs_msg);

    public:
        explicit Drone(const ros::NodeHandle& _nodeHandle);
        ~Drone();
        void _Teleop();
        void _Shutdown();
};
#include "../include/interface.h"
#include "keyboard.h"
#include "convert.h"

Interface::Interface(const ros::NodeHandle& _nodeHandle) :
    nh(_nodeHandle),

    pub_gcsRequest(nh.advertise<std_msgs::Header>("/drone/cmd/gcs", 1)),
    sub_droneRespond(nh.subscribe<std_msgs::Header>("/drone/cmd/drone", 10, &Interface::CallbackDroneRespond, this)),

    sub_state(nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &Interface::CallbackState, this)),
    sub_localPose(nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Interface::CallbackLocalPose, this)),
    sub_globalPose(nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &Interface::CallbackGlobalPose, this)),
    sub_setPose(nh.subscribe<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10, &Interface::CallbackSetPose, this)),

    timer_display(nh.createTimer(ros::Duration(1.0), &Interface::CallbackDisplay, this))

{
    system_status_map = {
        {0, "UNINIT"}, 
        {1, "BOOT"}, 
        {2, "CALIBRATING"}, 
        {3, "STANDBY"}, 
        {4, "ACTIVE"}, 
        {5, "CRITICAL"}, 
        {6, "EMERGENCY"}, 
        {7, "POWEROFF"}, 
        {8, "FLIGHT_TERMINATION"}
    };

    manual = {
        "+-----------------------------------------------------+",
        "|                   Control Pixhawk                   |",
        "|                                                     |",
        "|     1  2  3                                         |",
        "|     Q  W            G         U       O  P          |",
        "|     A  S  D                   J                     |",
        "|        X                                            |",
        "|                                                     |",
        "| Movement    : W / A / S / D / X                     |",
        "| Throttle    : U / J                                 |",
        "| Rotation    : 1 / 2 / 3                             |",
        "| Land        : G                                     |",
        "| Arm         : P                                     |",
        "| Offboard    : O                                     |",
        "| Quit        : Q                                     |",
        "+-----------------------------------------------------+"
    };

    info = {
        "+-----------------------------------------------------+",
        "                Pixhawk Information                    ",
        " Time [Now]    : ",
        " Status / Arm  : ",
        " Mode          : ",
        "-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -",
        " Local  pose   : ",
        " Global pose   : ",
        "-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -",
        " SetPose     : ",
        "-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -",
        " Time [Request] : ", " Request : ",
        " Time [Respond] : ", " Respond : ",
        " Time [Delay]   : ",
        " Message        : ",
        "+-----------------------------------------------------+"
    };

    _Teleop();
}

Interface::~Interface() { close_keyboard(); }

void Interface::PrintData()
{
    int index = 0;

    std::time_t t = (std::time_t)ros::Time::now().toSec();
    std::tm* tm = std::localtime(&t);

    std::time_t request_t = (std::time_t)request.stamp.toSec();
    std::time_t respond_t = (std::time_t)respond.stamp.toSec();


    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << info[index++] << '\n'
       << info[index++] << '\n' << '\n'
       << info[index++] << std::put_time(tm, "%Y-%m-%d %H:%M:%S") << '\n'
       << info[index++] << system_status_map.at(current_state.system_status) << " / " << (current_state.armed ? "Armed" : "Disarmed") << '\n'
       << info[index++] << current_state.mode << '\n'
       << info[index++] << '\n'
       << info[index++] << "[" << current_localPose.pose.position.x << ", " << current_localPose.pose.position.x << ", " << current_localPose.pose.position.x << "] / [" << current_yaw << "] \n"
       << info[index++] << "[La:" << current_globalPose.latitude << " / Lo:" << current_globalPose.longitude << "/ Al:" << current_globalPose.altitude << "]" << '\n'
       << info[index++] << '\n'
       << info[index++] << "[" << set_pose.pose.position.x << ", " << set_pose.pose.position.y << ", " << set_pose.pose.position.z << "] / [" << RadianToDegree(set_yaw) << "] \n"
       << info[index++] << '\n'
       << info[index++] << std::put_time(std::localtime(&request_t), "%Y-%m-%d %H:%M:%S") << info[index++] << (char)request.seq << '\n'
       << info[index++] << std::put_time(std::localtime(&respond_t), "%Y-%m-%d %H:%M:%S") << info[index++] << (char)respond.seq << '\n'
       << info[index++] << rtt_t << " [sec]" << '\n'
       << info[index++] << respond.frame_id << '\n'
       << info[index++] << '\n';

    std::cout << ss.str();
}


void Interface::PrintManual()
{
    for (const auto& line : manual) {
        std::cout << line << '\n';
    }
}

void Interface::CallbackDisplay(const ros::TimerEvent& event)
{
    PrintData();
}

void Interface::CallbackState(const mavros_msgs::State::ConstPtr& state_msg)
{
    current_state = *state_msg;
}


void Interface::CallbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    current_localPose = *pose_msg;
    
    
    auto e = QuaternionToEuler(current_localPose.pose.orientation.x, current_localPose.pose.orientation.y, current_localPose.pose.orientation.z, current_localPose.pose.orientation.w); 
    current_yaw = e[2];
}

void Interface::CallbackGlobalPose(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    current_globalPose = *gps_msg;
}

void Interface::CallbackSetPose(const geometry_msgs::PoseStamped::ConstPtr& setPose_msg)
{
    set_pose = *setPose_msg;
    auto e = QuaternionToEuler(set_pose.pose.orientation.x, set_pose.pose.orientation.y, set_pose.pose.orientation.z, set_pose.pose.orientation.w); 
    set_yaw = e[2];
}

void Interface::CallbackDroneRespond(const std_msgs::Header::ConstPtr& drone_msg)
{
    respond = *drone_msg;

    if (request.seq == respond.seq)
        rtt_t = respond.stamp.toSec() - request.stamp.toSec(); 
}


void Interface::_Teleop()
{
    ROS_INFO("Keyboard Teleop Start!");

    init_keyboard();    // 키보드 char 1개 입력받도록 터미널 설정 변경

    while (ros::ok())
    {
        if (_kbhit())
        {
            int ch = _getch();
            request.stamp = ros::Time::now();
            request.seq = ch;
            
            pub_gcsRequest.publish(request);

            respond.frame_id = "";
            rtt_t = -1;

            PrintData();

            if (ch == 'q')
            {
                ROS_INFO("Shutdown GCS node");
                close_keyboard();
                ros::shutdown();
            }

        }

        ros::spinOnce();
        rate.sleep();
    }

    return;
}

void Interface::_Shutdown()
{
    request.stamp = ros::Time::now();
    request.seq = 'q';
    
    pub_gcsRequest.publish(request);
    
    respond.frame_id = "";
    rtt_t = -1;

    ROS_INFO("Shutdown GCS node");
    close_keyboard();
    ros::shutdown();
}
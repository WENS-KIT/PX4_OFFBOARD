#include <drone.h>
#include "keyboard.h"

Drone::Drone(const ros::NodeHandle& _nodeHandle):
    nh(_nodeHandle),
    sub_state(nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &Drone::CallbackState, this)),
    sub_pose(nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Drone::CallbackPose, this)),
    
    pub_setpoint_local(nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10)),

    client_arm(nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming")),
    client_mode(nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode")),


    // Interface node <-> drone node
    sub_gcsRequest(nh.subscribe<std_msgs::Header>("/drone/cmd/gcs", 10, &Drone::CallbackGcsRequest, this)),
    pub_droneRespond(nh.advertise<std_msgs::Header>("/drone/cmd/drone", 10))

{
    if (ReadParam()) { _Shutdown(); return;}
    if (!CheckConnection()){ _Shutdown(); return; }
    
    TeleopMapping(teleop_map);

    if (enable_teleop)
        _Teleop();
    else
        while (ros::ok())
        {
            if (current_state.armed)
                Hover();

            ros::spinOnce();
            rate.sleep();
        }
}

Drone::~Drone() { close_keyboard(); }


void Drone::TeleopMapping(std::map<int, std::function<void()>>& keymap)
{
        // Task
    keymap['q'] = std::bind(&Drone::_Shutdown, this);
    keymap['g'] = std::bind(&Drone::Land, this);
    keymap['p'] = std::bind(&Drone::Arm, this);
    keymap['o'] = std::bind(&Drone::ChangeMode, this, "OFFBOARD");

    // Up & Down
    keymap['u'] = std::bind(&Drone::MoveLinear, this, 0, 0, 1);
    keymap['j'] = std::bind(&Drone::MoveLinear, this, 0, 0, -1);

    // Move
    keymap['w'] = std::bind(&Drone::MoveLinear, this, 1, 0, 0);
    keymap['a'] = std::bind(&Drone::MoveLinear, this, 0, -1, 0);
    keymap['s'] = std::bind(&Drone::MoveLinear, this, 0, 0, 0);
    keymap['d'] = std::bind(&Drone::MoveLinear, this, 0, 1, 0);
    keymap['x'] = std::bind(&Drone::MoveLinear, this, -1, 0, 0);

    // Turn
    keymap['1'] = std::bind(&Drone::MoveAngular, this, 10);
    keymap['3'] = std::bind(&Drone::MoveAngular, this, -10);
}

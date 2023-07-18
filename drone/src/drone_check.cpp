#include <drone.h>

template<typename T>
struct Param {
    std::string name;
    T& value;

    Param(std::string n, T& v) : name(n), value(v) {}
};

bool Drone::ReadParam() {
    bool check_error = false;

    // Add paramter here
    std::vector<Param<bool>> boolParams = {
        Param<bool>("enable_teleop", enable_teleop)
    };
    std::vector<Param<double>> doubleParams = {
        Param<double>("takeoff_height", takeoff_height)
    };

    for(auto& p : boolParams) {
        if(!nh.getParam(p.name, p.value)) {
            ROS_WARN("[FAIL] Default [%s : %d]", p.name.c_str(), p.value);
            check_error = true;
        } else {
            ROS_INFO("[READ] Get [%s : %d]", p.name.c_str(), p.value);
        }
    }

    for(auto& p : doubleParams) {
        if(!nh.getParam(p.name, p.value)) {
            ROS_WARN("[FAIL] Default [%s : %.2lf]", p.name.c_str(), p.value);
            check_error = true;
        } else {
            ROS_INFO("[READ] Get [%s : %.2lf]", p.name.c_str(), p.value);
        }
    }

    if (check_error) { ROS_WARN("Can't get parameters"); }
    return check_error;
}


bool Drone::CheckConnection()
{
    ROS_INFO("Check Connection...");

    int count = 0;

    ros::Time t_now = ros::Time::now();
    ros::Time t_prev = t_now;
    while (!current_state.connected)
    {
        t_now = ros::Time::now();
        if (t_now - t_prev > ros::Duration(2.0))
        {
            ROS_INFO("Waiting connect... %d", current_state.connected);
            count++;
            t_prev = t_now;
        }

        if (current_state.connected)
        {
            ROS_INFO("Connect success : %d", current_state.connected);
            return true;
        }

        if (count > 3)
        {
            ROS_ERROR("Connect Failed : %d", current_state.connected);
            return false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connect success : %d", current_state.connected);
    return true;
}


// Check Arm
bool Drone::CheckArm(bool state)
{
    if (current_state.armed == state) { 
        ROS_WARN("Already armed");  
        respond.frame_id = "Already armed";
        return true; 
    }
    return false;
}

// Check if Drone is flying
bool Drone::CheckFlying()
{
    // system_status 4 = MAV_STATE_ACTIVE
    if (current_state.armed == true && current_state.system_status == 4){
        return true;
    }
    
    ROS_WARN("Not flying");
    respond.frame_id = "Not flying";
    return false;
}

// Check if Mode is same with parameter
bool Drone::CheckMode(std::string mode)
{
    if (current_state.mode == mode)
        return true;

    return false;
}


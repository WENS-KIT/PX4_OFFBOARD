#include <drone.h>
#include <convert.h>

void Drone::Arm()
{
    if (CheckArm(true)) { return; }

    mavros_msgs::CommandBool cmd_arm;
    cmd_arm.request.value = true;

    if (!client_arm.call(cmd_arm)) { ROS_WARN("Can't transfer arming command"); respond.frame_id = "Can't transfer arming command";}
    else { ROS_INFO("Try arming ..."); respond.frame_id = "Try arming ...";}

}

// Change PX4 mode as parameter
void Drone::ChangeMode(std::string mode)
{
    if (CheckMode(mode)) { return ;}

    mavros_msgs::SetMode cmd_mode;
    cmd_mode.request.base_mode = 0;
    cmd_mode.request.custom_mode = mode;

    if (!client_mode.call(cmd_mode)) { 
        ROS_WARN("Can't tranfer mode command"); 
        respond.frame_id = "Can't tranfer mode command";
    }
    else { 
        ROS_INFO("Try to change mode to [%s]", mode.c_str()); 
        respond.frame_id = "Try to change mode to " + mode;
    }

}


void Drone::Land()
{
    if (!CheckFlying()) { return; }

    ChangeMode("AUTO.LAND"); // Change mode as AUTO.LAND
    
}

void Drone::Hover()
{
    ROS_INFO_DELAYED_THROTTLE(5, "hover... [%lf][%.2f, %.2f, %.2f]", RadianToDegree(current_yaw),
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

    pub_setpoint_local.publish(set_localPose);
}


// Set local position as current position
void Drone::SetCurrentPose()
{
    set_localPose.pose.position.x = current_pose.pose.position.x;
    set_localPose.pose.position.y = current_pose.pose.position.y;
    set_localPose.pose.position.z = current_pose.pose.position.z;
}

void Drone::SetCurrentYaw()
{
    set_localPose.pose.orientation.w = current_pose.pose.orientation.w;
    set_localPose.pose.orientation.x = current_pose.pose.orientation.x;
    set_localPose.pose.orientation.y = current_pose.pose.orientation.y;
    set_localPose.pose.orientation.z = current_pose.pose.orientation.z;
}


void Drone::MoveLinear(double pose_x, double pose_y, double pose_z)
{
    // Stop
    if (pose_x == 0 && pose_y == 0 && pose_z == 0)
    { SetCurrentPose(); }

    else
    {
        double goal_position[3] = {0, 0, pose_z};

        if (pose_x > 0)
        {
            goal_position[0] = pose_x * cos(current_yaw);
            goal_position[1] = pose_x * sin(current_yaw);
        }
        else if (pose_x < 0)
        {
            goal_position[0] = abs(pose_x) * -cos(current_yaw);
            goal_position[1] = abs(pose_x) * -sin(current_yaw);
        }

        if (pose_y > 0)
        {
            goal_position[0] = pose_y * sin(current_yaw);  //-cos(current_yaw + M_PI_2);
            goal_position[1] = pose_y * -cos(current_yaw); //-sin(current_yaw + M_PI_2);
        }
        else if (pose_y < 0)
        {
            goal_position[0] = abs(pose_y) * -sin(current_yaw); // cos(current_yaw + M_PI_2);
            goal_position[1] = abs(pose_y) * cos(current_yaw);  // sin(current_yaw + M_PI_2);
        }

        set_localPose.pose.position.x += goal_position[0];
        set_localPose.pose.position.y += goal_position[1];
        set_localPose.pose.position.z += goal_position[2];

        ROS_INFO("Move linear : %.2f, %.2f, %.2f", set_localPose.pose.position.x, set_localPose.pose.position.y, set_localPose.pose.position.z);

    }

    respond.frame_id = "Move Linear";
}

void Drone::MoveAngular(double degree)
{
    if (degree == 0)
        SetCurrentYaw();

    else
    {
        double goal_radian = current_yaw + DegreeToRadian(degree);

        if (goal_radian >= M_PI * 2)
            goal_radian = 0;

        current_yaw = goal_radian;

        auto quat = EulerToQauternion(0, 0, current_yaw);

        set_localPose.pose.orientation.x = quat[0];
        set_localPose.pose.orientation.y = quat[1];
        set_localPose.pose.orientation.z = quat[2];
        set_localPose.pose.orientation.w = quat[3];
        
        ROS_INFO("Move angular : [d: %lf][r: %lf]", RadianToDegree(current_yaw), current_yaw);

    }

    respond.frame_id = "Move Angular";
}
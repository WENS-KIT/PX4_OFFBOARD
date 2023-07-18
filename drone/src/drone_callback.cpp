#include <drone.h>
#include <convert.h>

void Drone::CallbackState(const mavros_msgs::State::ConstPtr& state_msg)
{
    if (current_state.mode != state_msg->mode){
        ROS_INFO("Mode Changed [%s->%s]", current_state.mode.c_str(), state_msg->mode.c_str());
        
        if (state_msg->mode == "OFFBOARD")
        {   
            // Flying & Arming+offboard => set current Position & set default height for takeoff
            if (current_state.armed){
                SetCurrentPose();
                SetCurrentYaw();
                
                // Check height and Set height
                if (current_pose.pose.position.z <= 0.4)
                    set_localPose.pose.position.z = takeoff_height;
            }
            else{
                // 
                SetCurrentPose();
                SetCurrentYaw();
                set_localPose.pose.position.z = takeoff_height;
            }
        }
    }    

    if (current_state.armed != state_msg->armed)
    {
        if (state_msg->armed)
            ROS_INFO("Now armed");
        else
            ROS_INFO("Now disarmed");
    }

    // Check failsafe
    if (state_msg->system_status >= 6){
        if (state_msg->system_status <= 8){
            ROS_ERROR("[EMERGENCY] Code Num : %d", state_msg->system_status);
            _Shutdown();
            return;
            
        }
        else{
            ROS_ERROR("[EMERGENCY] Dump Value : %d", state_msg->system_status);
            return;
        }
    }

    current_state = *state_msg;

}

void Drone::CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    current_pose = *pose_msg;
    
    auto e = QuaternionToEuler(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w); 
    current_yaw = e[2];
    
}


void Drone::CallbackGcsRequest(const std_msgs::Header::ConstPtr& gcs_msg)
{
    request = *gcs_msg;

    ROS_INFO("request message = %c", request.seq);

    if (request.seq > 0)
    {
        char key = (char)request.seq;
        respond.seq = request.seq;

        if (teleop_map.count(key) > 0)
        {
            teleop_map[key]();
            respond.stamp = ros::Time::now();
            pub_droneRespond.publish(respond);
        }
        
        else
        {
            ROS_INFO("Invalid Input");
            respond.stamp = ros::Time::now();
            respond.frame_id = "Invalid input";
        }
    }
    
}
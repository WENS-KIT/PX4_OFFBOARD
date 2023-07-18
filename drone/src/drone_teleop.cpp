/**
 * WASD : move
 * U/J  : up/down
 * T/G  : Takeoff/land(Ground)
 * O/P  : Offboard mode Arm / Change mode to Offboard
 * N/M  : Mission input / Mission mode Arm
 * 0    : On/Off tracking
 */


#include <drone.h>
#include <keyboard.h>


void Drone::_Teleop()
{   
    ROS_INFO("Keyboard Teleop Start!");

    init_keyboard();    // 키보드 char 1개 입력받도록 터미널 설정 변경

    while (ros::ok())
    {
        if (_kbhit())
        {
            int ch = _getch();
            if (teleop_map.count(ch) > 0) {
                teleop_map[ch]();
            }
        }

        if (current_state.armed)
            Hover();

        ros::spinOnce();
        rate.sleep();
    }

    return;
}

void Drone::_Shutdown()
{
    respond.stamp = ros::Time::now();
    respond.frame_id = "q";
    pub_droneRespond.publish(respond); // Transfer shutdown cmd to other node 

    ROS_INFO("Shutdown PX4 General");
    close_keyboard();
    ros::shutdown();
}
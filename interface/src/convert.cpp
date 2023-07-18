#include "../include/convert.h"


double DegreeToRadian(double degree) { return degree * M_PI / 180.0; }

double RadianToDegree (double radian) { return radian * 180.0 / M_PI; }

std::array<double, 3> QuaternionToEuler (double quat_x, double quat_y, double quat_z, double quat_w) 
{
    tf2::Quaternion q(quat_x, quat_y, quat_z, quat_w); 
    tf2::Matrix3x3 e(q);

    double tempRoll, tempPitch, tempYaw;
    e.getRPY(tempRoll, tempPitch, tempYaw);

    return {tempRoll, tempPitch, tempYaw};
}

std::array<double, 4> EulerToQauternion (double roll, double pitch, double yaw)
{
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);

    return {quat.getX(), quat.getY(), quat.getZ(), quat.getW()};
}
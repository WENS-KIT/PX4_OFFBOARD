#pragma once

#include <array>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

double DegreeToRadian(double degree);
double RadianToDegree (double radian);
std::array<double, 3> QuaternionToEuler (double, double, double, double);
std::array<double, 4> EulerToQauternion (double, double, double);
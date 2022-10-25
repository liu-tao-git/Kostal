/**
 * @example kstar.cpp
 * Select a plan from a list to execute using RDK's plan execution API.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Exception.hpp>
#include <Log.hpp>
#include <Scheduler.hpp>
#include <string>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include "ControlSPI.h"
#include <stdio.h>
#include <unistd.h>
#include <stack>
#include <array>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <math.h>
#include "math.h"
#include <eigen3/Eigen/Eigen>
using namespace std;


namespace {
    Eigen::Quaternion<double> q;
    struct Euler_out
    {
        double Roll,Pitch,Yaw;

    }g_euler_Out;

}


typedef struct Euler_out Struct;


Struct Quaternion_to_Euler (double w, double x, double y, double z)
{
    q.w() = w;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    // auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    auto euler = q.toRotationMatrix().eulerAngles(2,1,0);
    
    Struct Eout;
    {
    // Eout.Roll = euler[0];
    // Eout.Pitch = euler[1];
    // Eout.Yaw = euler[2];
    Eout.Roll = euler[2];
    Eout.Pitch = euler[1];
    Eout.Yaw = euler[0];
    }
    return Eout;
}


    
      

int main()
{
    g_euler_Out = Quaternion_to_Euler(0.523,-0.330,-0.754,-0.223);
    std::cout <<g_euler_Out.Roll<<","
              <<g_euler_Out.Pitch<<","
              <<g_euler_Out.Yaw
              <<std::endl;

}

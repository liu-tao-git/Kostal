/**
 * @example kstar.cpp
 * Select a plan from a list to execute using RDK's plan execution API.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>
#include <string>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include "ControlSPI.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // IP of the robot server
    std::string robotIP = "192.168.2.100";

    // IP of the workstation PC running this program
    std::string localIP = "192.168.2.104";

    // RDK Initialization
    //=============================================================================
    // RDK robot client
    auto robot = std::make_shared<flexiv::Robot>();

    // create data struct for storing robot states
    auto robotStates = std::make_shared<flexiv::RobotStates>();


    robot->init(robotIP, localIP);

    return 0;
}

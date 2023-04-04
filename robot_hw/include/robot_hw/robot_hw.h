#ifndef ROBOT_HW_H
#define ROBOT_HW_H

#include <iostream>
#include <string.h>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "../include/robot_hw/motor.h"

using namespace std;
using namespace std::chrono;

const string PORT_PREFIX = "/dev/ttyCH9344USB";

/**
 * @robot motor's layout:
 * float targets/data[12]
 *  
 */

enum MOTOR_LAYOUT{
    FL = 0,
    FR = 1,
    BR = 2,
    BL = 3
};

enum LEG_JOINTS{
    HIP = 0,
    AB = 1,
    KNEE = 2
};

class RobotHw {
    public:
        RobotHw() = default;

        ~RobotHw() {
            needExit = true;
            if (loopThread.joinable())
                loopThread.join();
        }

        bool init(ros::NodeHandle& nh);

        void update();

        void commandCallback(const std_msgs::Float32MultiArray& command) {
            ros_targets.data = command.data;
            //@todo: further processing the command position to be relatively with some zero positions???
        };
    
    private:
        std_msgs::Float32MultiArray ros_targets, ros_data;
        ros::Publisher dataPub;
        ros::Subscriber commandSub;
        serial serials[12];
        Motor* motors[12]; // Each of element points at the dynamically allocated motor object for the robot
        int updateFreq = 800; // default motor update frequency is 800Hz
        double loopTimeErrorThreshold = 0.00125; // the threshold is set to be 1.25ms
        high_resolution_clock::time_point lastUpdateTime;
        std::thread loopThread;
        bool isFirstUpdate = true;
        bool needExit = false;
};


#endif // end of the ifndef

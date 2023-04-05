#include "../include/robot_hw/robot_hw.h"

bool RobotHw::init(ros::NodeHandle& nh) {
    nh.getParam("/robot_hw/update_freqency", updateFreq);
    nh.getParam("/robot_hw/error_threshold", loopTimeErrorThreshold);

    cout<<"enter the init func"<<endl;
    for(int i = 0; i < 12; i++) {
        unsigned short port_id = (int(i / 6) * 8 + int(i % 6));
        string port = PORT_PREFIX + to_string((int)port_id); // @todo check with the real port number
        int comIdLength = port.length();
        char com_id[comIdLength + 1] = {0};
        strcpy(com_id, port.c_str());
        Motor* motor = new Motor(1, &serials[i], com_id);
        motors[i] = motor;
        cout<<"the "<<i<<"th motor's initialization with "<<com_id<<endl;
    }
    cout<<"finished motor initialization \n";
    // The second leg's ab joint's data is located at data[dim_stride[1] * 1 + 1] = data[4] 
    // ros_targets.layout.data_offset = ros_data.layout.data_offset = 0;
    // ros_targets.layout.dim[0].label = ros_data.layout.dim[0].label = "legs";
    // ros_targets.layout.dim[0].size = ros_data.layout.dim[0].size = 4;
    // ros_targets.layout.dim[0].stride = ros_data.layout.dim[0].stride = 4 * 3;
    // ros_targets.layout.dim[1].label = ros_data.layout.dim[1].label = "joints";
    // ros_targets.layout.dim[1].size = ros_data.layout.dim[1].size = 3;
    // ros_targets.layout.dim[1].stride = ros_data.layout.dim[1].stride = 3;
    cout<<"error0"<<endl;
    this->dataPub = nh.advertise<std_msgs::Float32MultiArray>("/robot/motor_data", 1);
    this->commandSub = nh.subscribe("/robot/motor_command", 1, &RobotHw::commandCallback, this);
    needExit = false;
    // set up update loop thread by the simple lambda expression
    loopThread = std::thread([&]() {
        while(!needExit)
            update();
    });
    cout<<"error"<<endl;
    sched_param sched{.sched_priority = 95};
    bool success = pthread_setschedparam(loopThread.native_handle(), SCHED_FIFO, &sched);
    cout<<"return value of set the thread priority: "<<success<<endl;
    if (success!= 0) {
        ROS_ERROR("Failed to set the robot hw update loop thread's priority");
        needExit = true;  
        return false;
    }

    isFirstUpdate = true;
    return true;
    cout<<"finishde init"<<endl;
}

void RobotHw::update() {
    high_resolution_clock::time_point currentTime = high_resolution_clock::now();
    // sync for the first time update
    if (isFirstUpdate) {
        lastUpdateTime = currentTime;
        isFirstUpdate = false;
    }

    duration<double> desiredUpdateDuration(1/updateFreq);
    duration<double> timeSpan = duration_cast<high_resolution_clock::duration>(currentTime - lastUpdateTime);
    double loopTimeError = (ros::Duration(timeSpan.count()) - ros::Duration(desiredUpdateDuration.count())).toSec();
    lastUpdateTime = high_resolution_clock::now();

    // if the update missed at least one update cycle
    if (loopTimeError > loopTimeErrorThreshold)
        ROS_WARN_STREAM("Robot hw update time error exceeds the threshold with the following setting: "
                        <<"error: "<<loopTimeError - loopTimeErrorThreshold
                        <<", loop time span: "<<timeSpan.count()
                        <<", threshold: "<<loopTimeErrorThreshold);
    
    // normal update loop time
    for (int i = 0; i < 12; i++) {
        motors[i]->setTarget(ros_targets.data[i], ControlMethod::POSITION);
        cout<<"entered the update func\n";
        ros_data.data[i] = motors[i]->getCurPos();
    }
    dataPub.publish(ros_data);

    // sleep
    high_resolution_clock::time_point desireEndingTime = currentTime 
        + duration_cast<high_resolution_clock::duration>(desiredUpdateDuration);
    std::this_thread::sleep_until(desireEndingTime);   
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_hw");
    ros::NodeHandle nh;
    cout<<"enter the node main func"<<endl;

    // a loop to update the hardware command
    // ros::AsyncSpinner asySpinner(1);
    // asySpinner.start();
    try {
        // initialize and update the hw
        RobotHw* robotHw = new RobotHw;
        robotHw->init(nh);
        // robotHw->update();
        // ros::waitForShutdown();
    } catch (const ros::Exception& e){
        ROS_FATAL_STREAM("Catched Error during init the robot hardware!! \n"
                        << "\t" <<"Exception: "<<e.what());
        return 1;
    }
    ros::spin();
    return 0;
}
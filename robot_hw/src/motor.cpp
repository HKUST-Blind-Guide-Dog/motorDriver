#include "../include/robot_hw/motor.h"
#include <iostream>

Motor::Motor(int id, serial* serialPort, char* com_id, int byte_size, int parity, int stop_bits, int baud_rate, int gearRatio, int encoderResolution)
:zeroPosition(0),  gearRatio(gearRatio), id(id), encoderResolution(encoderResolution)
{   
    //serial protocol initialization
    serialProtocol.serialPort = serialPort;
    serialProtocol.com_id = com_id;
    serialProtocol.byte_size = byte_size;
    serialProtocol.parity = parity;
    serialProtocol.baud_rate = baud_rate;
    serialProtocol.stop_bits = stop_bits;
    if (!serial_open(serialProtocol.serialPort, serialProtocol.com_id, serialProtocol.baud_rate, 
    serialProtocol.byte_size, serialProtocol.parity, serialProtocol.stop_bits)) 
        isConnected = 1;
    else 
        isConnected = 0;
    printf("whether successful open the serial port: %d \n", isConnected);
    setTarget(0, ControlMethod::VELOCITY); // set the motor's initial state to be zero velocity
    int initialzeCounter = 0;
    while(!readMOtorData()){
        initialzeCounter++;
    }
    printf("The number needed to initialize the serial port is %d!", initialzeCounter);
    setId(id);
    setZero(position);
}

bool Motor::setId(int idValue)
{
    const int bufferSize = 5 + 8;
    uint8_t commandBuffer[bufferSize] = {0};
    commandBuffer[0] = 0x3E; // msg header
    commandBuffer[1] = 0xCD;
    commandBuffer[2] = 8; // data size
    commandBuffer[3] = 0x79;
    commandBuffer[4] = commandBuffer[6] = commandBuffer[7] = commandBuffer[8] = commandBuffer[9] = 0x00;
    commandBuffer[5] = 0; // for write
    commandBuffer[10] = idValue;

    // serial communicatiom
    serial_crc16(commandBuffer, bufferSize - 2, &commandBuffer[11], &commandBuffer[12]); // write the corresponding crc
    serial_write(serialProtocol.serialPort, bufferSize, commandBuffer);
    //for debugging
    if (IS_DEBUG){
        printf("set the motor id to %d \n", idValue);
        printf("command bffer: ");
        for (int i = 0; i < bufferSize; i++) {
            printf("%x", commandBuffer[i]);
            printf("\t");
        }
        printf("\n");
    }
    uint8_t dataBuffer[bufferSize];
    serial_read(serialProtocol.serialPort, bufferSize, dataBuffer);
    
    if (dataBuffer[10] == idValue) {
        id = idValue;
        if (IS_DEBUG)
            printf("successfully set the ID!!! \n");
        return true;
    }else 
        return false;
}

bool Motor::setComSpeed(int speed)
{
    const int bufferSize = 5 + 8;
    uint8_t commandBuffer[bufferSize] = {0};
    commandBuffer[0] = 0x3E; // msg header
    commandBuffer[1] = id;
    commandBuffer[2] = 8; // data size
    commandBuffer[3] = 0xB4;
    commandBuffer[4] = commandBuffer[5] = commandBuffer[6] = commandBuffer[7] 
    = commandBuffer[8] = commandBuffer[9] =  0x00;

    switch (speed)
    {
        case 115200:
            commandBuffer[10] = 0x00;
            break;

        case 500000:
            commandBuffer[10] = 0x01;
            break;

        case 1000000:
            commandBuffer[10] = 0x02;
            break;

        case 1500000:
            commandBuffer[10] = 0x03;
            break;

        case 2500000:
            commandBuffer[10] = 0x04;
            break;
        
        default:
            printf("the speed is not the available for the motor!! \n");
            return false;
            break;
    }

    // serial communicatiom
    serial_crc16(commandBuffer, bufferSize - 2, &commandBuffer[11], &commandBuffer[12]); // write the corresponding crc
    serial_write(serialProtocol.serialPort, bufferSize, commandBuffer);
    // for debugging
    if (IS_DEBUG){
        printf("change the motor speed!! \n");
        printf("command bffer: ");
        for (int i = 0; i < bufferSize; i++) {
            printf("%x", commandBuffer[i]);
            printf("\t");
        }
        printf("\n");
    }
    uint8_t dataBuffer[bufferSize] = {0};
    serial_read(serialProtocol.serialPort, bufferSize, dataBuffer);
    bool isMotorSet = 0;
    for (int i = 0; i < bufferSize; i++)
        if (dataBuffer[i] != 0) {
            isMotorSet = 1;
            break;
        }

    bool serialSetResult = 0;
    if (isMotorSet)
        serialSetResult = serial_change_baudrate(serialProtocol.serialPort, speed);
    if (!serialSetResult)
        serialProtocol.baud_rate = speed;
    printf("current speed: %d \n", serialProtocol.baud_rate);
    return !serialSetResult ? true : false;
}

void Motor::setTarget(float target, ControlMethod controlMode) 
{   
    const int bufferSize = 5 + 8;
    uint8_t commandBuffer[bufferSize] = {0};
    commandBuffer[0] = 0x3E; // msg header
    commandBuffer[1] = id;
    commandBuffer[2] = 8; // data size

    // close loop control
    commandBuffer[3] = 0xA0 + controlMode; // specify whther it is torque/ vel/ position
    commandBuffer[4] = commandBuffer[5] = commandBuffer[6] = commandBuffer[9] = commandBuffer[10] = 0;

    if (controlMode == ControlMethod::TORQUE) 
    {
        int16_t tor = (int16_t)(tor * 100);
        commandBuffer[7] = (uint8_t)(tor & 0xFF);
        commandBuffer[8] = (uint8_t)((tor >> 8) & 0xFF);
    }else{
        int value = (int)(target * 100);
        commandBuffer[7] = (uint8_t)(value & 0xFF);
        commandBuffer[8] = (uint8_t)((value >> 8) & 0xFF);
        commandBuffer[9] = (uint8_t)((value >> 16) & 0xFF);
        commandBuffer[10] = uint8_t((value >> 24) & 0xFF);
    }

    // serial communicatiom
    serial_crc16(commandBuffer, bufferSize - 2, &commandBuffer[11], &commandBuffer[12]); // write the corresponding crc
    serial_write(serialProtocol.serialPort, bufferSize, commandBuffer);
    // for debugging
    if (IS_DEBUG) {
        printf("command bffer: ");
        for (int i = 0; i < bufferSize; i++) {
            printf("%x", commandBuffer[i]);
            printf("\t");
        }
        printf("\n");
    }
    uint8_t dataBuffer[bufferSize];
    serial_read(serialProtocol.serialPort, bufferSize, dataBuffer);

    if (IS_DEBUG) {
        printf("data buffer: ");
        for (int i = 0; i < bufferSize; i++)
            printf("%x \t", dataBuffer[i]);
        printf("\n");
    }
    
    // decoding the data
    // torque = (int16_t)(dataBuffer[6] << 8 + dataBuffer[5]) * 33.0 / 2048;
    // velocity = (int16_t)(dataBuffer[8] << 8 + dataBuffer[7]) * 1.0 / gearRatio;
    // position = (int16_t)(dataBuffer[10] << 8 + dataBuffer[9]) * 360.0 / gearRatio / encoderResolution;
    // strictly follow the protocol description to decode
    torque = (int16_t)(dataBuffer[6] << 8 + dataBuffer[5]) / 100;
    velocity = (int16_t)(dataBuffer[8] << 8 + dataBuffer[7]) * 1.0;
    position = (int16_t)(dataBuffer[10] << 8 + dataBuffer[9]) * 1.0;
}

bool Motor::readMOtorData()
{
    const int bufferSize = 5 + 8;
    uint8_t commandBuffer[bufferSize] = {0};
    commandBuffer[0] = 0x3E; // msg header
    commandBuffer[1] = this->id;
    commandBuffer[2] = 8; // data size

    // send zero torque command to the motor
    commandBuffer[3] = 0xA1;
    // commandBuffer[3] = 0xA3;
    // commandBuffer[4] = 0x64;
    for (int i = 4; i < 11; i++)
        commandBuffer[i] = 0x00;
    serial_crc16(commandBuffer, bufferSize - 2, &commandBuffer[11], &commandBuffer[12]); // write the corresponding crc
    serial_write(serialProtocol.serialPort, bufferSize, commandBuffer);
    uint8_t data[bufferSize] = {0};
    int success = serial_read(this->serialProtocol.serialPort, bufferSize, data);
    if (IS_DEBUG) {
        printf("command buffer: ");
        for (int i = 0; i < bufferSize; i++) 
            printf("%x \t", commandBuffer[i]);
        printf("\n");
        printf("data buffer: ");
        for (int i = 0; i < bufferSize; i++) 
            printf("%x \t", data[i]);
        printf("\n");   
    }
    torque = (int16_t)(data[6] << 8 + data[5]) / 100;
    velocity = (int16_t)(data[8] << 8 + data[7]) * 1.0;
    position = (int16_t)(data[10] << 8 + data[9]) * 1.0;
    if (IS_DEBUG)
        printf("decoded data: \n torque:%f, velocity:%f, position:%f \n", torque, velocity, position);
    return (success == 0) ? true : false;
}


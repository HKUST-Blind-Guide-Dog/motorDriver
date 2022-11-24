#include "motor.h"

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

    // setId(id);
    setTarget(0, ControlMethod::VELOCITY); // set the motor's initial state to be zero velocity
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
    uint8_t dataBuffer[bufferSize];
    serial_read(serialProtocol.serialPort, bufferSize, dataBuffer);
    
    if (dataBuffer[10] == idValue) {
        id = idValue;
        return true;
    }else 
        return false;
}

void Motor::setComSpeed(int speed)
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

        case 2000000:
            commandBuffer[10] = 0x04;
            break;
        
        default:
            break;
    }

    // serial communicatiom
    serial_crc16(commandBuffer, bufferSize - 2, &commandBuffer[11], &commandBuffer[12]); // write the corresponding crc
    serial_write(serialProtocol.serialPort, bufferSize, commandBuffer);
    uint8_t dataBuffer[bufferSize];
    serial_read(serialProtocol.serialPort, bufferSize, dataBuffer);
}

void Motor::setTarget(int target, ControlMethod controlMode) 
{   
    const int bufferSize = 5 + 8;
    uint8_t commandBuffer[bufferSize] = {0};
    commandBuffer[0] = 0x3E; // msg header
    commandBuffer[1] = id;
    commandBuffer[2] = 8; // data size

    // close loop control
    commandBuffer[3] = 0xA0 + controlMode; // specify whther it is torque/ vel/ position
    commandBuffer[4] = commandBuffer[5] = commandBuffer[6] = commandBuffer[9] = commandBuffer[10] = 0;
    switch(controlMode) {
        case ControlMethod::TORQUE:
            int16_t tor = (int16_t)(tor * 100);
            commandBuffer[7] = (uint8_t)(tor & 0xFF);
            commandBuffer[8] = (uint8_t)((tor >> 8) & 0xF0);
            break;

        case ControlMethod::VELOCITY:
        case ControlMethod::POSITION:
            int value = (int)(target * 100 * gearRatio);
            commandBuffer[7] = (uint8_t)(value & 0xFF);
            commandBuffer[8] = (uint8_t)((value >> 8) & 0xF0);
            commandBuffer[9] = (uint8_t)((value >> 16) & 0xFF);
            commandBuffer[10] = uint8_t((value >> 24) & 0xFF);
            break;
            
        default: break;
    }

    // serial communicatiom
    serial_crc16(commandBuffer, bufferSize - 2, &commandBuffer[11], &commandBuffer[12]); // write the corresponding crc
    serial_write(serialProtocol.serialPort, bufferSize, commandBuffer);
    uint8_t dataBuffer[bufferSize];
    serial_read(serialProtocol.serialPort, bufferSize, dataBuffer);
    
    // decoding the data
    torque = (int16_t)(dataBuffer[6] << 8 + dataBuffer[5]) * 33.0 / 2048;
    velocity = (int16_t)(dataBuffer[8] << 8 + dataBuffer[7]) * 1.0 / gearRatio;
    position = (int16_t)(dataBuffer[10] << 8 + dataBuffer[9]) * 360.0 / gearRatio / encoderResolution;
}


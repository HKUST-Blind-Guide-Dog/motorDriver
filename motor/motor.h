#ifndef __MOTOR__H__
#define __MOTOR__H__

#include "../curi_serial/serial.h"

// char* comId = "/dev/ttyUSB0";
#define IS_DEBUG 1
#define bufferSize (5 + 8) 

enum ControlMethod{
    POSITION = 3,
    VELOCITY = 2,
    TORQUE = 1
};

struct SerialInfo{
    serial* serialPort;
    char* com_id;
    int baud_rate;
    int byte_size;
    int parity; 
    int stop_bits;
};

class Motor{
    public:
        Motor() = default;

        Motor(int id, serial* serialPort, char* com_id = "/dev/ttyCH9344USB1", int byte_size = 8, 
        int parity = 0, int stop_bits = 1, int baud_rate = 2500000, int gearRatio = 9, int encoderResolution = 16384);

        ~Motor() { serial_close(serialProtocol.serialPort); }

        void setZero();

        void setTarget(float target, ControlMethod controlMode);

        bool setId(int idValue);

        bool setComSpeed(int speed);

        bool readMOtorData();

        //accesor
        bool isMotorConnected() const { return isConnected; }
        float getZeroPosition() const { return zeroPosition; } 
        float getCurPos() const { return position; }
        float getCurVel() const { return velocity; }
        float getCurTor() const { return torque; }

    private:
        bool isConnected;
        float zeroPosition;
        float position;
        float velocity;
        float torque;
        int gearRatio;
        int encoderResolution;

        // for serial communication
        SerialInfo serialProtocol;
        int id;
        
        uint8_t commandBuffer[bufferSize];
        uint8_t dataBuffer[bufferSize];
};

#endif // end of the ifndef

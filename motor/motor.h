#ifndef __MOTOR__H__
#define __MOTOR__H__

#include "../curi_serial/serial.h"

// char* comId = "/dev/ttyUSB0";

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
        Motor(int id, serial* serialPort, char* com_id = "/dev/ttyUSB0", int byte_size = 8, 
        int parity = 0, int stop_bits = 1, int baud_rate = 115200, int gearRatio = 9, int encoderResolution = 16384);

        ~Motor() { serial_close(serialProtocol.serialPort); }

        void setZero(float zeroPosition) { this->zeroPosition = zeroPosition; }

        void setTarget(int target, ControlMethod controlMode);

        bool setId(int idValue);

        void setComSpeed(int speed);

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
};

#endif // end of the ifndef

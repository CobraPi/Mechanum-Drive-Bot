#ifndef MechanumDrive_H
#define MechanumDrive_H

#include <pin_map.h>
#include <Encoder.h>//
#include <AFMotor.h>

#define FL_A_PIN 7
#define FL_B_PIN 8

enum DriveState {CAR, STRAFE};

enum DriveMode {HEADING_HOLD, ROTATE};

class MechanumDrive {
    public:
        MechanumDrive();

    protected:
        Motor m_fl(FL_A_PIN, FL_B_PIN); // Front left M_RL
        Motor m_fr()

    private:
        DriveState _state;
        uint16_t _speed;
        uint16_t _currentHeading, _targetHeading;

        Encoder _frontLeftEncoder;
        Encoder _rearLeftEncoder;
        Encoder _frontRightEncoder;
        
        
}








#endif // _
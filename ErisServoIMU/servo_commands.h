#ifndef SERVO_COMMANDS_H
#define SERVO_COMMANDS_H

#include <modules/serialcom.h>

namespace SerialCom {
    void ServoMove();
    void ServoSmoothMove();

    // Trayectorias
    void TrajChannels();
    void TrajPeriod();
    void TrajPoint();
    void TrajGo();
    void TrajEnd();
    void TrajStop();
    void TrajStatus();

    // IMUs
    void ImuZero();
    void ImuPrint();
}

#endif

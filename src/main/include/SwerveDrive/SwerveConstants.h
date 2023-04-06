#pragma once

#include <string>
#include <cmath>

#include <frc2/command/PIDCommand.h>

#include "Geometry/Point.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif

namespace SwerveConstants{

    struct SwerveStruct{
        std::string name;
        int driveID;
        int turnID;
        int encoderID;
        double encoderOffset;
        Point pos; //Meters
        frc::PIDController turnPID = {0, 0, 0};
    };

    const SwerveStruct FL = {   "Front Left",       //Name
                                23,                 //Drive ID
                                5,                  //Turn ID
                                2, 175.8,           //Encoder ID, offset
                                {-0.3429, 0.3429}   //Position
                            };

    const SwerveStruct FR = {   "Front Right",
                                4,
                                10,
                                8, 173.75,
                                {0.3429, 0.3429}
                            };

    const SwerveStruct BL = {   "Back Left",
                                22,
                                19,
                                6, 41.6,
                                {-0.3429, -0.3429}
                            };

    const SwerveStruct BR = {   "Back Right",
                                1,
                                7,
                                9, 74.26,
                                {0.3429, -0.3429}
                            };

    const SwerveStruct MODULES[] = {FL, FR, BL, BR};
    const int NUMSWERVE = 4;

    const double DRIVE_MAX_VOLTS = 3.0; //Volts
    const double TURN_MAX_VOLTS = 3.0; //Volts

    const double WHEEL_RADIUS = 0.0508; //Meters
    const double TICKS_PER_ROTATION = 2048.0;
    const double TICKS_PER_RADIAN = (TICKS_PER_ROTATION / (2.0*M_PI));
};
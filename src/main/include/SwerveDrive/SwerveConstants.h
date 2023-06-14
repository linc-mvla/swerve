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
        int turnEncoderID;
        int driveEncoderID[2];
        double encoderOffset;
        Point pos; //Meters
        frc::PIDController turnPID = {2.7, 0, 0};
    };

    //X axis is the side
    //Y axis is front-back
    const SwerveStruct FL = {   "Front Left",       //Name
                                2,                 //Drive ID
                                1,                  //Turn ID
                                2, {6,7}, -185.009 + 180.0,        //turnID, drive encoder ID, offset
                                {-0.3429, 0.3429}   //Position
                            };

    const SwerveStruct FR = {   "Front Right",
                                21,
                                22,
                                1, {8,9}, -9.31,
                                {0.3429, 0.3429}
                            };

    const SwerveStruct BL = {   "Back Left",
                                11,
                                12,
                                0, {2,3}, -98.613,
                                {-0.3429, -0.3429}
                            };

    const SwerveStruct BR = {   "Back Right",
                                32,
                                31,
                                3, {4,5}, -209.855 + 180.0,
                                {0.3429, -0.3429}
                            };

    const SwerveStruct MODULES[] = {FL, FR, BL, BR};
    const int NUMSWERVE = 4;

    const double DRIVE_MAX_VOLTS = 3.0; //Volts
    const double TURN_MAX_VOLTS = 12.0; //Volts

    const double WHEEL_RADIUS = 0.0508; //Meters
    const double TICKS_PER_ROTATION = 2048.0;
    const double TICKS_PER_RADIAN = (TICKS_PER_ROTATION / (2.0*M_PI));
};
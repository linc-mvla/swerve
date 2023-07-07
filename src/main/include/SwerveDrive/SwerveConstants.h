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
        bool inverted = false; //Pos turn volts = neg encoder value
        frc::PIDController turnPID = {6.7, 0, 0};
    };

    //X axis is the side
    //Y axis is front-back
    const SwerveStruct FL = {   "Front Left",       //Name
                                1,                 //Drive ID
                                2,                  //Turn ID
                                0, {6,7}, -1.673,        //turnID, drive encoder ID, offset
                                {-0.3429, 0.3429},   //Position
                                false                    //inverted
                            };

    const SwerveStruct FR = {   "Front Right",
                                21,
                                22,
                                2, {8,9}, 1.735,
                                {0.3429, 0.3429}
                            };

    const SwerveStruct BL = {   "Back Left",
                                11,
                                12,
                                1, {2,3}, -1.297,
                                {-0.3429, -0.3429}
                            };

    const SwerveStruct BR = {   "Back Right",
                                31,
                                32,
                                3, {4,5}, -1.374,
                                {0.3429, -0.3429}
                            };

    const SwerveStruct MODULES[] = {FL, FR, BL, BR};
    const int NUMSWERVE = 4;

    const double DRIVE_MAX_VOLTS = 6.0; //Volts
    const double TURN_MAX_VOLTS = 12.0; //Volts

    const double WHEEL_RADIUS = 0.0508; //Meters
    const double TICKS_PER_ROTATION = 2048.0;
    const double TICKS_PER_RADIAN = (TICKS_PER_ROTATION / (2.0*M_PI));
};
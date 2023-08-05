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
        double encoderOffset; //Degrees
        Point pos; //Meters
        bool encoderInverted = true; //If the motor/wheel spins opposite to the encoder
        frc::PIDController turnPID = {4.0, 0, 0.2}; // Radians -> volts
    };
    const std::string canBus = "Drivebase";
    //X axis is the side-side axis, back is negative, forwards is positive
    //Y axis is the front-back axis, left is negative, right is positive
    //This way an angle of 90 is to go forward
    const double HALFSIDE_LENGTH = 0.3429;
    const SwerveStruct FL = {   "Front Left",       //Name
                                21,                 //Drive ID
                                15,                 //Turn ID
                                62, 107.1,          //Encoder ID, offset added to read value
                                {HALFSIDE_LENGTH, -HALFSIDE_LENGTH}   //Position
                            };

    const SwerveStruct FR = {   "Front Right",
                                14,
                                13,
                                42, -161.99,
                                {HALFSIDE_LENGTH, HALFSIDE_LENGTH}
                            };

    const SwerveStruct BL = {   "Back Left",
                                17,
                                18,
                                8, 109.96,
                                {-HALFSIDE_LENGTH, -HALFSIDE_LENGTH}
                            };

    const SwerveStruct BR = {   "Back Right",
                                11,
                                12,
                                10, 5.2,
                                {-HALFSIDE_LENGTH, HALFSIDE_LENGTH}
                            };

    const SwerveStruct MODULES[] = {FL, FR, BL, BR};
    const int NUMSWERVE = 4;

    const double DRIVE_MAX_VOLTS = 3.0; //Volts
    const double TURN_MAX_VOLTS = 12.0; //Volts

    const double WHEEL_RADIUS = 0.0508; //Meters
    const double TICKS_PER_ROTATION = 2048.0;
    const double TICKS_PER_RADIAN = (TICKS_PER_ROTATION / (2.0*M_PI));
};
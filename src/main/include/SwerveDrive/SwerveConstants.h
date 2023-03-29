#pragma once

#include <string>
#include <cmath>
#include "Geometry/Point.h"

namespace SwerveConstants{

    struct SwerveStruct{
        std::string name;
        int driveID;
        int turnID;
        Point pos; //Meters
    };

    const SwerveStruct FL = {   "Front Left",
                                0,
                                0,
                                {0.0, 0.0}
                            };

    const SwerveStruct FR = {   "Front Right",
                                0,
                                0,
                                {0.0, 0.0}
                            };

    const SwerveStruct BL = {   "Back Left",
                                0,
                                0,
                                {0.0, 0.0}
                            };

    const SwerveStruct BR = {   "Back Right",
                                0,
                                0,
                                {0.0, 0.0}
                            };

    const SwerveStruct MODULES[] = {FL, FR, BL, BR};
    const int NUMSWERVE = 4;

    const double DRIVE_MAX_VOLTS = 3.0; //Volts

    const double WHEEL_RADIUS = 2.0; //Meters
    const double TICKS_PER_ROTATION = 2048.0;
    const double TICKS_PER_RADIAN = TICKS_PER_ROTATION / (2.0*M_PI);
};
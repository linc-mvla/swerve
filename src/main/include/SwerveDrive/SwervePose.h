#pragma once

#include "Geometry/Point.h"

namespace SwervePose{
    struct SwervePose{
        Point pos;
        Vector vel;
        Vector accel;

        double ang; //Radians
        double angVel;
        double angAccel;
    };

    void zero(SwervePose pose){
        pose.pos = {0.0,0.0};
        pose.vel = {0.0,0.0};
        pose.accel = {0.0,0.0};
        pose.ang = 0.0;
        pose.angVel = 0.0;
        pose.angAccel = 0.0;
    }
}
#pragma once

#include "Controls/Controls.h"

Vector Controls::getStrafe(){
    double x = rJoy_.GetRawAxis(ControlConstants::JOY_X);
    double y = -rJoy_.GetRawAxis(ControlConstants::JOY_Y);
    Vector p{x, y};
    if(p.originDist() < ControlConstants::RJOY_DEADBAND){
        return Point(0.0, 0.0);
    }
    return p;
}

double Controls::getRotation(){
    double w = lJoy_.GetRawAxis(ControlConstants::JOY_X);
    if(abs(w) < ControlConstants::LJOY_DEADBAND){
        return 0.0;
    }
    return w;
}

bool Controls::isZero(){
    return rJoy_.GetRawButton(ControlConstants::ZERO);
}
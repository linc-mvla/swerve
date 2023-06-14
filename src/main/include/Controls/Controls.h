#pragma once

#include <frc/Joystick.h>

#include "Geometry/Point.h"
#include "ControlConstants.h"

class Controls{
    public:
        Vector getStrafe();
        double getRotation();
        bool fieldOrientPressed();
        
    private:
        frc::Joystick lJoy_{ControlConstants::LJOY_PORT};
        frc::Joystick rJoy_{ControlConstants::RJOY_PORT};
};
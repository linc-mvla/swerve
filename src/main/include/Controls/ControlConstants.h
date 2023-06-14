#pragma once

namespace ControlConstants{
    const int LJOY_PORT = 0;
    const int RJOY_PORT = 1;

    const int JOY_X = 0;
    const int JOY_Y = 1;

    //LJOY
    const double LJOY_DEADBAND = 0.05;

    const double STRAFE_CONST = 12.0;

    //RJOY
    const double RJOY_DEADBAND = 0.05;

    const double ROTATION_CONST = 12.0; //How much to increase rotation

    //Field Orient
    const int FIELD_ORIENT = 0; //LJOY
}
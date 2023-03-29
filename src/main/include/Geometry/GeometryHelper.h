#pragma once

#include <cmath>

namespace GeometryHelper{
    double toDeg(double rad){
        return rad / M_PI * 180.0;
    }

    double toRad(double deg){
        return deg * M_PI / 180.0;
    }
}
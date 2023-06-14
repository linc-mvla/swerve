#pragma once

#include "ShuffleboardItem.h"
#include "SwerveDrive/SwervePose.h"

template class ShuffleboardItem<float>;
template class ShuffleboardItem<bool>;
template class ShuffleboardItem<int>;
template class ShuffleboardItem<units::volt_t>;
template class ShuffleboardItem<frc::PIDController>;
//template class ShuffleboardItem<SwervePose::ModulePose>;
//template class ShuffleboardItem<SwervePose::Pose>;
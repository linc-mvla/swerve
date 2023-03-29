#pragma once

#include <string>
#include <array>

#include <AHRS.h>

#include "SwerveModule.h"
#include "SwerveConstants.h"
#include "SwervePose.h"

class SwerveDrive{
    public:
        SwerveDrive(std::string name);
        void reset();
        void zero();

        void Periodic();
        void TeleopPeriodic();
        void DisabledPeriodic();

        SwervePose::SwervePose getCurrPose();
        
        void setNAVX(AHRS* navx) {navx_ = navx;}

    private:
        void updatePose();

        std::string name_;
        SwerveModule modules_[SwerveConstants::NUMSWERVE];

        SwervePose::SwervePose targetPose_;
        SwervePose::SwervePose currentPose_;

        double lastUpdate_;

        AHRS* navx_;
};
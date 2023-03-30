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

        void drive();
        
        void SetTarget(Vector v, double angV, bool volts = true);

        SwervePose::Pose getCurrPose();
        
        void setNAVX(AHRS* navx) {navx_ = navx;}

    private:
        void updatePose();

        std::string name_;
        SwerveModule modules_[SwerveConstants::NUMSWERVE];

        Point pivot_{0.0, 0.0};

        SwervePose::Pose targetPose_;
        SwervePose::Pose currentPose_;
        bool volts_ = true;

        double lastUpdate_;

        AHRS* navx_;
};
#pragma once

#include <string>
#include <array>
#include <iostream>

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
        void TeleopInit();
        void TeleopPeriodic();
        void DisabledInit();
        void DisabledPeriodic();

        void drive();
        
        void SetTarget(Vector v, double angV, bool volts = true);

        SwervePose::Pose getCurrPose();
        
        void setNAVX(AHRS* navx) {navx_ = navx;}

        void enableShuffleboard(bool edit = false, bool module = false);
        void disableSuffleboard();

    private:
        void updatePose();

        const std::string name_;
        SwerveModule* modules_[SwerveConstants::NUMSWERVE];

        Point pivot_{0.0, 0.0};

        SwervePose::Pose targetPose_;
        SwervePose::Pose currentPose_;
        bool volts_ = true;

        double lastUpdate_;

        AHRS* navx_;

        struct ShuffleboardData{
            bool initialized = false;
            bool showDashboard = false;
            bool edit = false;
            frc::ShuffleboardTab* tab;
            nt::GenericEntry *currAng, *currAngV, *currAngAccel,
                             *currPosX, *currVX, *currXAccel,
                             *currPosY, *currVY, *currYAccel,
                             *targetVY, *targetVX, *targetVAng,
                             *volts;
        };

        ShuffleboardData shuffData_;
        void printShuffleboard();
};
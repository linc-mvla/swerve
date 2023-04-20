// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include <string>
#include <iostream>

#include <AHRS.h>

#include "RobotConstants.h"
#include "SwerveDrive/SwerveDrive.h"

#define USE_CONTROLLER false
#if USE_CONTROLLER
#include "Controls/Controls.h"
#endif

class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;
        void RobotPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void TestInit() override;
        void TestPeriodic() override;
        void SimulationInit() override;
        void SimulationPeriodic() override;

    private:
        SwerveDrive drive_{""};
        AHRS* navx_;
        #if USE_CONTROLLER
        Controls controls_;
        #endif
};

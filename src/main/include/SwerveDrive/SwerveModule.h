#pragma once

#include <string>
#include <cmath>
#include <ctre/Phoenix.h>
#include <units/voltage.h>

#include <frc2/command/PIDCommand.h>

#include "Geometry/Point.h"
#include "SwervePose.h"
#include "SwerveConstants.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif

class SwerveModule{
    public:
        SwerveModule();
        SwerveModule(SwerveConstants::SwerveStruct swerveMod);
        SwerveModule(SwerveModule& other) : name_(other.name_),
                                            pos_(other.pos_),
                                            driveMotor_(other.driveMotor_),
                                            turnMotor_(other.turnMotor_),
                                            turnPID_(other.turnPID_)
                                            {};

        void Periodic();
        void TeleopPeriodic();
        void DisabledPeriodic();

        void setTarget(SwervePose::ModulePose pose, bool volts = true);

        void ShowShuffleBoard();

        Point getPos();
        Vector getVel();

        SwerveModule& operator= (const SwerveModule& module);

    private:
        std::string name_;

        WPI_TalonFX* driveMotor_;
        units::volt_t driveVolts_{0.0};
        
        WPI_TalonFX* turnMotor_;
        units::volt_t turnVolts_{0.0};

        frc::PIDController turnPID_;
        SwervePose::ModulePose targetPose_;
        bool volts_;

        Point pos_; //Position on robot

        double wheelAng_; // Rad
        double wheelVel_; // meters/s
        Vector vel_;

        bool inverted_;
};
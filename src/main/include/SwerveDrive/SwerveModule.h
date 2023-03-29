#pragma once

#include <string>
#include <cmath>
#include <ctre/Phoenix.h>
#include <units/voltage.h>

#include "Geometry/Point.h"
#include "SwerveConstants.h"

class SwerveModule{
    public:
        SwerveModule(std::string name, int driveID, int turnID, Point pos);
        SwerveModule() = default;
        SwerveModule(SwerveModule& other) : name_(other.name_), pos_(other.pos_), driveMotor_(other.driveMotor_), turnMotor_(other.turnMotor_){};

        void Periodic();
        void TeleopPeriodic();
        void DisabledPeriodic();

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

        Point pos_; //Position on robot

        double wheelAng_; // Rad
        double wheelVel_; // meters/s
        Vector vel_;

        bool inverted_;
};
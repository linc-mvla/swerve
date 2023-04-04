#pragma once

#include <string>
#include <cmath>
#include <ctre/Phoenix.h>
#include <units/voltage.h>

#include <frc2/command/PIDCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "Geometry/Point.h"
#include "SwervePose.h"
#include "SwerveConstants.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif

class SwerveModule{
    public:
        SwerveModule() = default;
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

        void zero();

        void setTarget(SwervePose::ModulePose pose, bool volts = true);

        void enableShuffleboard(bool edit = false);
        void disableSuffleboard();

        Point getPos();
        Vector getVel();

        SwerveModule& operator= (const SwerveModule& module);

    private:
        std::string name_;

        WPI_TalonFX* driveMotor_;
        units::volt_t driveVolts_{0.0};
        double maxDriveVolts_ = SwerveConstants::DRIVE_MAX_VOLTS;//volts
        
        WPI_TalonFX* turnMotor_;
        units::volt_t turnVolts_{0.0};
        double maxTurnVolts_ = SwerveConstants::TURN_MAX_VOLTS;

        frc::PIDController turnPID_{0, 0, 0};
        SwervePose::ModulePose targetPose_;//either m/s or volts for drive
        bool volts_; //using volts or nah

        SwervePose::ModulePose currPose_; //normal units (m, rad)
        Vector vel_;

        Point pos_; //Position on robot, accessed by swerveDrive, stored in module

        bool inverted_;
        
        struct ShuffleboardData{
            bool initialized = false;
            bool showDashboard = false;
            bool edit = false;
            frc::ShuffleboardTab* tab;
            nt::GenericEntry *driveVolts, *turnVolts,
                             *maxTurn, *maxDrive,
                             *turnP, *turnI, *turnD,
                             *targAng, *targVel, *targVolts,
                             *currAng, *currVel,
                             *inverted;
        };

        ShuffleboardData shuffData_;
        void printShuffleboard();
};
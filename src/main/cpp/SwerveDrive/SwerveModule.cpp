#include "SwerveDrive/SwerveModule.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveModule::SwerveModule(SwerveConstants::SwerveStruct swerveMod):
    name_(swerveMod.name),
    driveMotor_(swerveMod.driveID, "drivebase"),
    turnMotor_(swerveMod.turnID, "drivebase"),
    turnEncoder_(swerveMod.turnEncoderID),
    driveEncoder_(swerveMod.driveEncoderID[0],swerveMod.driveEncoderID[1]),
    turnPID_(swerveMod.turnPID),
    pos_(swerveMod.pos),
    encoderOffset_(swerveMod.encoderOffset),
    ShuffData_(name_)
{
    driveMotor_.SetNeutralMode(NeutralMode::Coast);
    turnMotor_.SetNeutralMode(NeutralMode::Coast);

}

void SwerveModule::Periodic(){
    //Calc velocity
    //double wheelAng = turnMotor_->GetSelectedSensorPosition() / SwerveConstants::TICKS_PER_RADIAN;
    double wheelAng = toRad(turnEncoder_.GetAbsolutePosition() + encoderOffset_);
    if(inverted_){
        wheelAng += M_PI;
    }
    double driveAngVel = driveMotor_.GetSelectedSensorVelocity() * 10.0 / SwerveConstants::TICKS_PER_RADIAN;
    double wheelVel = driveAngVel * SwerveConstants::WHEEL_RADIUS;
    currPose_.ang = wheelAng;
    currPose_.speed = wheelVel;
    vel_ = Vector(cos(wheelAng) * wheelVel , sin(wheelAng) * wheelVel);

    ShuffData_.update();
}

void SwerveModule::TeleopInit(){
    driveMotor_.SetNeutralMode(NeutralMode::Brake);
    turnMotor_.SetNeutralMode(NeutralMode::Brake);
};

void SwerveModule::TeleopPeriodic(){
    double driveTarg = std::clamp(targetPose_.speed, -maxDriveVolts_, maxDriveVolts_);
    if(inverted_){
        driveTarg = -driveTarg;
    }
    driveVolts_ = units::volt_t(driveTarg);
    driveMotor_.SetVoltage(driveVolts_);

    double turnDiff = getAngDiff(targetPose_.ang, currPose_.ang);
    if(turnDiff > M_PI/2.0){
        inverted_ = !inverted_;
        turnDiff = -M_PI + turnDiff;
    }
    else if(turnDiff < -M_PI/2.0){
        inverted_ = !inverted_;
        turnDiff = M_PI + turnDiff;
    }
    double turnTarg = turnPID_.Calculate(turnDiff);
    turnTarg = std::clamp(turnTarg, -maxTurnVolts_, maxTurnVolts_);
    turnVolts_ = units::volt_t(turnTarg);
    turnMotor_.SetVoltage(turnVolts_);
}

void SwerveModule::DisabledInit(){
    driveMotor_.SetNeutralMode(NeutralMode::Coast);
    turnMotor_.SetNeutralMode(NeutralMode::Coast);
}

void SwerveModule::DisabledPeriodic(){
    driveMotor_.SetVoltage(units::volt_t{0.0});
    turnMotor_.SetVoltage(units::volt_t{0.0});
}

void SwerveModule::zero(){
    turnMotor_.SetSelectedSensorPosition(0.0);
}

void SwerveModule::setTarget(SwervePose::ModulePose pose, bool volts){
    targetPose_ = pose;
    volts_ = volts;
}

void SwerveModule::enableShuffleboard(bool edit){
    if(ShuffData_.isInitialized()){
        ShuffData_.enable(edit);
    }
    ShuffData_.Initialize(edit);
    ShuffData_.add("Drive Volts", &driveVolts_, {1,1,0,0});
    ShuffData_.add("Turn Volts", &turnVolts_, {1,1,1,0});
    ShuffData_.add("Drive Max Volts", &maxDriveVolts_, {1,1,0,1}, edit = true);
    ShuffData_.add("Turn Max Volts", &maxTurnVolts_, {1,1,1,1}, edit = true);
    ShuffData_.add("Turn PID", &turnPID_, {1,2,6,0}, edit = true);
    ShuffData_.add("Target Pose", &targetPose_, {2,2,2,0});
    ShuffData_.add("Volts", &volts_, {1,1,2,2});
    ShuffData_.add("Current Pose", &currPose_, {2,2,4,0});
    ShuffData_.add("Inverted", &inverted_, {1,1,4,2});
}

void SwerveModule::disableSuffleboard(){
    ShuffData_.disable();
}

std::string SwerveModule::getName(){
    return name_;
}

Point SwerveModule::getPos(){
    return pos_;
}

Vector SwerveModule::getVel(){
    return vel_;
}
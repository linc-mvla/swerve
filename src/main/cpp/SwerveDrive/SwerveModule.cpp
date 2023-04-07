#include "SwerveDrive/SwerveModule.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveModule::SwerveModule(SwerveConstants::SwerveStruct swerveMod):
    name_(swerveMod.name),
    pos_(swerveMod.pos),
    turnPID_(swerveMod.turnPID),
    encoderOffset_(swerveMod.encoderOffset)
    #ifdef SHUFFLEBOARD_PRINT
    ,ShuffData_(swerveMod.name)
    #endif
{
    driveMotor_ = new WPI_TalonFX(swerveMod.driveID);
    turnMotor_ = new WPI_TalonFX(swerveMod.turnID);
    cancoder_ = new WPI_CANCoder(swerveMod.encoderID);

    driveMotor_->SetNeutralMode(NeutralMode::Coast);
    turnMotor_->SetNeutralMode(NeutralMode::Coast);

    #ifdef SHUFFLEBOARD_PRINT
    ShuffData_.add("Current Pose", &currPose_);
    #endif
}

void SwerveModule::Periodic(){
    //Calc velocity
    //double wheelAng = turnMotor_->GetSelectedSensorPosition() / SwerveConstants::TICKS_PER_RADIAN;
    double wheelAng = toRad(cancoder_->GetAbsolutePosition() + encoderOffset_);
    if(inverted_){
        wheelAng += M_PI;
    }
    double driveAngVel = driveMotor_->GetSelectedSensorVelocity() * 10.0 / SwerveConstants::TICKS_PER_RADIAN;
    double wheelVel = driveAngVel * SwerveConstants::WHEEL_RADIUS;
    currPose_.ang = wheelAng;
    currPose_.speed = wheelVel;
    vel_ = Vector(cos(wheelAng) * wheelVel , sin(wheelAng) * wheelVel);
}

void SwerveModule::TeleopInit(){
    driveMotor_->SetNeutralMode(NeutralMode::Brake);
    turnMotor_->SetNeutralMode(NeutralMode::Brake);
};

void SwerveModule::TeleopPeriodic(){
    double driveTarg = std::clamp(targetPose_.speed, -maxDriveVolts_, maxDriveVolts_);
    driveVolts_ = units::volt_t(driveTarg);
    driveMotor_->SetVoltage(driveVolts_);

    double turnDiff = getAngDiff(targetPose_.ang, currPose_.ang);
    if(turnDiff > M_PI/2.0){
        inverted_ = true;
        turnDiff = -M_PI + turnDiff;
    }
    else if(turnDiff < -M_PI/2.0){
        inverted_ = true;
        turnDiff = M_PI + turnDiff;
    }
    double turnTarg = turnPID_.Calculate(turnDiff);
    turnTarg = std::clamp(turnTarg, -maxTurnVolts_, maxTurnVolts_);
    turnVolts_ = units::volt_t(turnTarg);
    turnMotor_->SetVoltage(turnVolts_);
}

void SwerveModule::DisabledInit(){
    driveMotor_->SetNeutralMode(NeutralMode::Coast);
    turnMotor_->SetNeutralMode(NeutralMode::Coast);
}

void SwerveModule::DisabledPeriodic(){
    driveMotor_->SetVoltage(units::volt_t{0.0});
    turnMotor_->SetVoltage(units::volt_t{0.0});
}

void SwerveModule::zero(){
    turnMotor_->SetSelectedSensorPosition(0.0);
}

void SwerveModule::setTarget(SwervePose::ModulePose pose, bool volts){
    targetPose_ = pose;
    volts_ = volts;
}

Point SwerveModule::getPos(){
    return pos_;
}

Vector SwerveModule::getVel(){
    return vel_;
}

SwerveModule& SwerveModule::operator= (const SwerveModule& module){
    name_ = module.name_;
    driveMotor_ = module.driveMotor_;
    turnMotor_ = module.turnMotor_;
    pos_ = module.pos_;
    return *this;
}
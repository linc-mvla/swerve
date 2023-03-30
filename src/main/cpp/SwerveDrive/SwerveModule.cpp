#include "SwerveDrive/SwerveModule.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveModule::SwerveModule():
    turnPID_(0, 0, 0)
{

}

SwerveModule::SwerveModule(SwerveConstants::SwerveStruct swerveMod):
    name_(swerveMod.name),
    pos_(swerveMod.pos),
    turnPID_(swerveMod.turnPID)
{
    driveMotor_ = new WPI_TalonFX(swerveMod.driveID);
    turnMotor_ = new WPI_TalonFX(swerveMod.turnID);
}

void SwerveModule::Periodic(){
    //Calc velocity
    wheelAng_ = turnMotor_->GetSelectedSensorPosition() / SwerveConstants::TICKS_PER_RADIAN;
    if(inverted_){
        wheelAng_ += M_PI;
    }
    double angVel = driveMotor_->GetSelectedSensorVelocity() * 10.0 / SwerveConstants::TICKS_PER_RADIAN;
    wheelVel_ = angVel * SwerveConstants::WHEEL_RADIUS;
    vel_ = Vector(cos(wheelAng_) * wheelVel_ , sin(wheelAng_) * wheelVel_);
}

void SwerveModule::TeleopPeriodic(){
    double driveTarg = std::clamp(targetPose_.speed, -SwerveConstants::DRIVE_MAX_VOLTS, SwerveConstants::DRIVE_MAX_VOLTS);
    driveVolts_ = units::volt_t(driveTarg);
    driveMotor_->SetVoltage(driveVolts_);

    double turnDiff = getAngDiff(targetPose_.ang, wheelAng_);
    if(turnDiff > M_PI/2.0){
        inverted_ = true;
        turnDiff = -M_PI + turnDiff;
    }
    else if(turnDiff < -M_PI/2.0){
        inverted_ = true;
        turnDiff = M_PI + turnDiff;
    }
    double turnTarg = turnPID_.Calculate(turnDiff);
    turnTarg = std::clamp(turnTarg, -SwerveConstants::TURN_MAX_VOLTS, SwerveConstants::TURN_MAX_VOLTS);
    turnVolts_ = units::volt_t(turnTarg);
    turnMotor_->SetVoltage(turnVolts_);
}

void SwerveModule::DisabledPeriodic(){
    driveMotor_->SetVoltage(units::volt_t{0.0});
    turnMotor_->SetVoltage(units::volt_t{0.0});
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
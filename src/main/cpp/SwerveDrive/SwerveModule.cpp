#include "SwerveDrive/SwerveModule.h"

SwerveModule::SwerveModule(std::string name, int driveID, int turnID, Point pos):
    name_(name),
    pos_(pos)
{
    driveMotor_ = new WPI_TalonFX(driveID);
    turnMotor_ = new WPI_TalonFX(turnID);
}

void SwerveModule::Periodic(){
    //Calc velocity
    wheelAng_ = turnMotor_->GetSelectedSensorPosition() / SwerveConstants::TICKS_PER_RADIAN;
    double angVel = driveMotor_->GetSelectedSensorVelocity() * 10.0 / SwerveConstants::TICKS_PER_RADIAN;
    wheelVel_ = angVel * SwerveConstants::WHEEL_RADIUS;
    vel_ = Vector(cos(wheelAng_) * wheelVel_ , sin(wheelAng_) * wheelVel_);
}

void SwerveModule::TeleopPeriodic(){
    driveMotor_->SetVoltage(driveVolts_);
    turnMotor_->SetVoltage(turnVolts_);
}

void SwerveModule::DisabledPeriodic(){
    driveMotor_->SetVoltage(units::volt_t{0.0});
    turnMotor_->SetVoltage(units::volt_t{0.0});
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
#include "SwerveDrive/SwerveDrive.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveDrive::SwerveDrive(std::string name):
    name_(name)
{
    for(int i = 0; i < SwerveConstants::NUMSWERVE; i++){
        modules_[i] = SwerveModule(SwerveConstants::MODULES[i]);
        modules_[i].enableShuffleboard();
    }
    reset();
}

void SwerveDrive::reset(){
    zero();
    SwervePose::zero(targetPose_);
}

void SwerveDrive::zero(){
    SwervePose::zero(currentPose_);
    navx_->ZeroYaw();
    for(SwerveModule& module : modules_){
        module.zero();
    }
}

void SwerveDrive::Periodic(){
    for(SwerveModule& module : modules_){
        module.Periodic();
    }
    updatePose();
}

void SwerveDrive::updatePose(){
    double time = frc::Timer::GetFPGATimestamp().value();
    double dt = time - lastUpdate_;

    double newAng = toRad(navx_->GetYaw());
    double newAngVel = (newAng - currentPose_.ang) / dt;
    double newAngAccel = (newAngVel - currentPose_.angVel) / dt;
    currentPose_.ang = newAng;
    currentPose_.angVel = newAngVel;
    currentPose_.angAccel = newAngAccel;

    Vector velocity{0.0, 0.0};
    for(SwerveModule& module : modules_){
        velocity += module.getVel();
    }
    velocity /= SwerveConstants::NUMSWERVE;

    velocity.rotateThis(-currentPose_.ang);

    currentPose_.pos += velocity * dt;
    currentPose_.accel = (velocity - currentPose_.vel) / dt;
    currentPose_.vel = velocity;
}

void SwerveDrive::TeleopPeriodic(){
    for(SwerveModule& module : modules_){
        module.TeleopPeriodic();
    }
    drive();
}

void SwerveDrive::drive(){
    for(SwerveModule& module : modules_){
        Vector angVelVec = module.getPos() - pivot_;
        angVelVec.rotateClockwise90This();
        Vector moduleVec = targetPose_.vel.rotate(-currentPose_.ang) + (angVelVec*targetPose_.ang);
        module.setTarget(SwervePose::ModulePose{moduleVec.originDist(), moduleVec.getAng()});
    }
}

void SwerveDrive::DisabledPeriodic(){
    for(SwerveModule& module : modules_){
        module.DisabledPeriodic();
    }
}

void SwerveDrive::enableShuffleboard(bool edit){
    shuffData_.showDashboard = true;
    shuffData_.edit = edit;
    if(shuffData_.initialized){
        return;
    }
    shuffData_.initialized = true;
    shuffData_.tab = &frc::Shuffleboard::GetTab(name_ + " Swerve Drive");
}

void SwerveDrive::disableSuffleboard(){
    shuffData_.showDashboard = false;
}

void SwerveDrive::SetTarget(Vector v, double angV, bool volts){
    targetPose_.vel = v;
    targetPose_.angVel = angV;
    volts_ = volts;
}

SwervePose::Pose SwerveDrive::getCurrPose(){
    return currentPose_;
}
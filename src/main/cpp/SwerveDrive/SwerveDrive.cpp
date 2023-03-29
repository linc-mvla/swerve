#include "SwerveDrive/SwerveDrive.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveDrive::SwerveDrive(std::string name):
    name_(name)
{
    for(int i = 0; i < SwerveConstants::NUMSWERVE; i++){
        SwerveConstants::SwerveStruct module = SwerveConstants::MODULES[i];
        modules_[i] = SwerveModule( module.name, 
                                    module.driveID,
                                    module.turnID,
                                    module.pos
                                    );
    }

    reset();
}

void SwerveDrive::reset(){
    zero();
    SwervePose::zero(targetPose_);
}

void SwerveDrive::zero(){
    SwervePose::zero(currentPose_);
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
}

void SwerveDrive::DisabledPeriodic(){
    for(SwerveModule& module : modules_){
        module.DisabledPeriodic();
    }
}

SwervePose::SwervePose SwerveDrive::getCurrPose(){
    return currentPose_;
}
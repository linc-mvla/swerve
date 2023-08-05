#include "SwerveDrive/SwerveDrive.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveDrive::SwerveDrive(std::string name):
    name_(name)
{
    for(int i = 0; i < SwerveConstants::NUMSWERVE; i++){
        modules_[i] = new SwerveModule(SwerveConstants::MODULES[i]);
    }
}

void SwerveDrive::reset(){
    zero();
    SwervePose::zero(targetPose_);
}

void SwerveDrive::zero(){
    SwervePose::zero(currentPose_);
    if(navx_){
        navx_->ZeroYaw();
    }
}

void SwerveDrive::Periodic(){
    for(SwerveModule* module : modules_){
        module->Periodic();
    }
    //updatePose();
    //printShuffleboard();
}

void SwerveDrive::updatePose(){
    double time = frc::Timer::GetFPGATimestamp().value();
    double dt = time - lastUpdate_;

    if(navx_){
        double newAng = toRad(navx_->GetYaw());
        double newAngVel = (newAng - currentPose_.ang) / dt;
        double newAngAccel = (newAngVel - currentPose_.angVel) / dt;
        currentPose_.ang = newAng;
        currentPose_.angVel = newAngVel;
        currentPose_.angAccel = newAngAccel;
    }

    //Average the velocities
    Vector velocity{0.0, 0.0};
    for(SwerveModule* module : modules_){
        velocity += module->getVel();
    }
    velocity /= SwerveConstants::NUMSWERVE;

    //Field-Orient
    velocity.rotateThis(-currentPose_.ang);

    currentPose_.pos += velocity * dt;
    currentPose_.accel = (velocity - currentPose_.vel) / dt;
    currentPose_.vel = velocity;
}

void SwerveDrive::TeleopInit(){
    for(SwerveModule* module : modules_){
        module->TeleopInit(); //Motors to brake to not be scooted by enemy
    }
}

void SwerveDrive::TeleopPeriodic(){
    drive(); //Set module targets
    for(SwerveModule* module : modules_){
        module->TeleopPeriodic(); //Drive motors and stuff
    }
}

/***
 * Based off the targetPose_, it will assign a target position for the modules
 * i.e. tell what the modules to do
*/
void SwerveDrive::drive(){
    for(SwerveModule* module : modules_){
        Vector angVelVec = module->getPos() - pivot_; //Get vector from pivot to module
        angVelVec.rotateCounterclockwise90This(); //Set to vector tangent to the path of rotation

        //Adds tangential velocity, which is just the target tangential velocity (rotated by the robot's pose)
        //Adds the rotational velocity, which is the angVelVec times the angular velocity: v = r*w
        Vector moduleVec = targetPose_.vel.rotate(-currentPose_.ang) + (angVelVec*targetPose_.angVel);

        //std::cout<<"Target for "<<module->getName()<<": "<<moduleVec.toString()<<std::endl;
        double speed = moduleVec.originDist();
        if(speed != 0){//Atan2 of 0,0 is very funky
            module->setTarget(SwervePose::ModulePose{speed, moduleVec.getAng()});
        }
        else{
            module->setTarget(SwervePose::ModulePose{0.0, 0.0}); //Maybe lock wheels? maybe after some time
        }
    }
}

void SwerveDrive::DisabledInit(){
    for(SwerveModule* module : modules_){
        module->DisabledInit(); //Sets the motor mode to coast (so robot can be scooted by ppl)
    }
}

void SwerveDrive::DisabledPeriodic(){
    for(SwerveModule* module : modules_){
        module->DisabledPeriodic(); //Tell robot to actively do nothing
    }
}

/***
 * Enables shuffleboard prints
 * @param edit can edit target values
 * @param modules enable module prints
*/
void SwerveDrive::enableShuffleboard(bool edit, bool modules){
    shuffData_.showDashboard = true;
    shuffData_.edit = edit;
    if(modules){
        for(SwerveModule* module : modules_){
            module->enableShuffleboard(edit);
        }
    }
    if(shuffData_.initialized){
        return;
    }
    shuffData_.initialized = true;
    shuffData_.tab = &frc::Shuffleboard::GetTab(name_ + " Swerve Drive");
    shuffData_.currAng = shuffData_.tab->Add("Angle", toDeg(currentPose_.ang)).GetEntry();
    shuffData_.currAngV = shuffData_.tab->Add("Angle V", toDeg(currentPose_.angVel)).GetEntry();
    shuffData_.currAngAccel = shuffData_.tab->Add("Angle Accel", toDeg(currentPose_.angAccel)).GetEntry();
    shuffData_.currPosX = shuffData_.tab->Add("Pos X", currentPose_.pos.getX()).GetEntry();
    shuffData_.currPosX = shuffData_.tab->Add("Pos Y", currentPose_.pos.getY()).GetEntry();
    shuffData_.currVX = shuffData_.tab->Add("Vel X", currentPose_.vel.getX()).GetEntry();
    shuffData_.currVY = shuffData_.tab->Add("Vel Y", currentPose_.vel.getY()).GetEntry();
    shuffData_.currXAccel = shuffData_.tab->Add("Accel X", currentPose_.accel.getX()).GetEntry();
    shuffData_.currYAccel = shuffData_.tab->Add("Accel Y", currentPose_.accel.getY()).GetEntry();
    shuffData_.targetVX = shuffData_.tab->Add("Target Vel X", targetPose_.vel.getX()).GetEntry();
    shuffData_.targetVY = shuffData_.tab->Add("Target Vel Y", targetPose_.vel.getY()).GetEntry();
    shuffData_.targetVAng = shuffData_.tab->Add("Target Ang Vel", toDeg(targetPose_.angVel)).GetEntry();
    shuffData_.volts = shuffData_.tab->Add("Volts", volts_).GetEntry();
}

void SwerveDrive::disableSuffleboard(){
    shuffData_.showDashboard = false;
}

void SwerveDrive::printShuffleboard(){
    if((!shuffData_.showDashboard) || (!shuffData_.initialized)){
        std::cout<<"Skipped"<<std::endl;
        return;
    }
    shuffData_.currAng->SetDouble(toDeg(currentPose_.ang));
    shuffData_.currAngV->SetDouble(toDeg(currentPose_.angVel));
    shuffData_.currAngAccel->SetDouble(toDeg(currentPose_.angAccel));
    shuffData_.currPosX->SetDouble(currentPose_.pos.getX());
    shuffData_.currPosY->SetDouble(currentPose_.pos.getY());
    shuffData_.currVX->SetDouble(currentPose_.vel.getX());
    shuffData_.currVY->SetDouble(currentPose_.vel.getY());
    shuffData_.currXAccel->SetDouble(currentPose_.accel.getX());
    shuffData_.currYAccel->SetDouble(currentPose_.accel.getY());
    if(shuffData_.edit){
        targetPose_.vel.setX(shuffData_.targetVX->GetDouble(targetPose_.vel.getX()));
        targetPose_.vel.setY(shuffData_.targetVY->GetDouble(targetPose_.vel.getY()));
        targetPose_.angVel = toRad(shuffData_.targetVAng->GetDouble(targetPose_.angVel));
    }
    shuffData_.targetVX->SetDouble(targetPose_.vel.getX());
    shuffData_.targetVY->SetDouble(targetPose_.vel.getY());
    shuffData_.targetVAng->SetDouble(toDeg(targetPose_.angVel));
}

void SwerveDrive::SetTarget(Vector v, double angV, bool volts){
    targetPose_.vel = v;
    targetPose_.angVel = angV;
    volts_ = volts;
}

SwervePose::Pose SwerveDrive::getCurrPose(){
    return currentPose_;
}
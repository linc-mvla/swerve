#include "SwerveDrive/SwerveModule.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveModule::SwerveModule(SwerveConstants::SwerveStruct swerveMod):
    name_(swerveMod.name),
    driveMotor_(swerveMod.driveID, "drivebase"),
    turnMotor_(swerveMod.turnID, "drivebase"),
    cancoder_(swerveMod.encoderID, "drivebase"),
    pos_(swerveMod.pos),
    turnPID_(swerveMod.turnPID),
    encoderOffset_(swerveMod.encoderOffset)
{
    driveMotor_.SetNeutralMode(NeutralMode::Coast);
    turnMotor_.SetNeutralMode(NeutralMode::Coast);
    if(shuffData_.showDashboard){
        enableShuffleboard();
    }
}

void SwerveModule::Periodic(){
    //Calc velocity
    //double wheelAng = turnMotor_->GetSelectedSensorPosition() / SwerveConstants::TICKS_PER_RADIAN;
    double wheelAng = toRad(cancoder_.GetAbsolutePosition() + encoderOffset_);
    if(inverted_){
        wheelAng += M_PI;
    }
    double driveAngVel = driveMotor_.GetSelectedSensorVelocity() * 10.0 / SwerveConstants::TICKS_PER_RADIAN;
    double wheelVel = driveAngVel * SwerveConstants::WHEEL_RADIUS;
    currPose_.ang = wheelAng;
    currPose_.speed = wheelVel;
    vel_ = Vector(cos(wheelAng) * wheelVel , sin(wheelAng) * wheelVel);

    printShuffleboard();
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
    shuffData_.showDashboard = true;
    shuffData_.edit = edit;
    if(shuffData_.initialized){
        return;
    }
    shuffData_.initialized = true;
    shuffData_.tab = &frc::Shuffleboard::GetTab(name_ + " Swerve Module");
    shuffData_.driveVolts = shuffData_.tab->Add("Drive Volts", driveVolts_.value()).GetEntry();
    shuffData_.turnVolts = shuffData_.tab->Add("Turn Volts", turnVolts_.value()).GetEntry();
    shuffData_.maxDrive = shuffData_.tab->Add("Drive Max Volts", maxDriveVolts_).GetEntry();
    shuffData_.maxTurn = shuffData_.tab->Add("Turn Max Volts", maxTurnVolts_).GetEntry();
    shuffData_.turnP = shuffData_.tab->Add("P", turnPID_.GetP()).GetEntry();
    shuffData_.turnI = shuffData_.tab->Add("I", turnPID_.GetI()).GetEntry();
    shuffData_.turnD = shuffData_.tab->Add("D", turnPID_.GetD()).GetEntry();
    shuffData_.targAng = shuffData_.tab->Add("Target Ang", toDeg(targetPose_.ang)).GetEntry();
    shuffData_.targVel = shuffData_.tab->Add("Target Vel", targetPose_.speed).GetEntry();
    shuffData_.targVolts = shuffData_.tab->Add("Target Volts", volts_).GetEntry();
    shuffData_.currAng = shuffData_.tab->Add("Ang", toDeg(currPose_.ang)).GetEntry();
    shuffData_.currVel = shuffData_.tab->Add("Vel", currPose_.speed).GetEntry();
    shuffData_.inverted = shuffData_.tab->Add("Inverted", inverted_).GetEntry();
}

void SwerveModule::disableSuffleboard(){
    shuffData_.showDashboard = false;
}

void SwerveModule::printShuffleboard(){
    if(!shuffData_.showDashboard){
        return;
    }
    //std::cout<<name_<<"; Angle:"<<targetPose_.ang<<"; Speed:"<<targetPose_.speed<<std::endl;
    shuffData_.driveVolts->SetDouble(driveVolts_.value());
    shuffData_.turnVolts->SetDouble(turnVolts_.value());
    shuffData_.currAng->SetDouble(toDeg(currPose_.ang));
    shuffData_.currVel->SetDouble(currPose_.speed);
    if(shuffData_.edit){
        maxDriveVolts_ = shuffData_.maxDrive->GetDouble(maxDriveVolts_);
        maxTurnVolts_ = shuffData_.maxTurn->GetDouble(maxDriveVolts_);
        turnPID_.SetP(shuffData_.turnP->GetDouble(turnPID_.GetP()));
        turnPID_.SetI(shuffData_.turnI->GetDouble(turnPID_.GetI()));
        turnPID_.SetD(shuffData_.turnD->GetDouble(turnPID_.GetD()));
        //targetPose_.ang = shuffData_.targAng->GetDouble(toRad(targetPose_.ang));
        //targetPose_.speed = shuffData_.targVel->GetBoolean(targetPose_.speed);
        volts_ = shuffData_.targVolts->GetBoolean(volts_);
    }
    shuffData_.targAng->SetDouble(toDeg(targetPose_.ang));
    shuffData_.targVel->SetDouble(targetPose_.speed);
    shuffData_.targVolts->SetBoolean(volts_);
    shuffData_.inverted->SetBoolean(inverted_);
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
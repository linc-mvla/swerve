#include "ShuffleboardSender/ShuffleboardSender.h"

ShuffleboardSender::ShuffleboardSender(std::string name):
name_(name)
{
}

void ShuffleboardSender::Initialize(bool edit){
    edit_ = edit;
    enabled_ = true;
    if(!initialized_){
        tab_ = &frc::Shuffleboard::GetTab(name_);
        initialized_ = true;
    }
}

void ShuffleboardSender::add(ShuffleboardItem item){
    items_.push_back(item);
}

void ShuffleboardSender::add(std::string name, double* o, bool edit){
    items_.push_back(ShuffleboardItem({name, tab_, edit}, o));
}
void ShuffleboardSender::add(std::string name, bool* o, bool edit){
    items_.push_back(ShuffleboardItem({name, tab_, edit}, o));
}
void ShuffleboardSender::add(std::string name, int* o, bool edit){
    items_.push_back(ShuffleboardItem({name, tab_, edit}, o));
}
void ShuffleboardSender::add(std::string name, units::volt_t* o, bool edit){
    items_.push_back(ShuffleboardItem({name, tab_, edit}, o));
}
void ShuffleboardSender::add(std::string name, frc::PIDController* o, bool edit){
    items_.push_back(ShuffleboardItem({name, tab_, edit}, o));
}
void ShuffleboardSender::add(std::string name, SwervePose::Pose* o, bool edit){
    items_.push_back(ShuffleboardItem({name, tab_, edit}, o));
}

void ShuffleboardSender::update(){
    if(edit_){
        for(int i = 0; i< items_.size(); i++){
        items_[i].edit();
    }
    for(int i = 0; i< items_.size(); i++){
        items_[i].send();
    }
};
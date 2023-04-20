#include "ShuffleboardSender/ShuffleboardSender.h"

ShuffleboardSender::ShuffleboardSender(std::string name):
name_(name)
{
}

void ShuffleboardSender::Initialize(bool edit){
    edit_ = edit;
    if(!initialized_){
        tab_ = &frc::Shuffleboard::GetTab(name_);
        initialized_ = true;
    }
}

void ShuffleboardSender::add(ShuffleboardItem item){
    items_.push_back(item);
}

void ShuffleboardSender::add(std::string name, double* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}
void ShuffleboardSender::add(std::string name, bool* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}
void ShuffleboardSender::add(std::string name, int* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}
void ShuffleboardSender::add(std::string name, units::volt_t* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}
void ShuffleboardSender::add(std::string name, frc::PIDController* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}

void ShuffleboardSender::send(){
    int size = items_.size();
    for(int i = 0; i<size; i++){
        items_[i].send();
    }
};

void ShuffleboardSender::edit(){
    int size = items_.size();
    for(int i = 0; i<size; i++){
        items_[i].edit();
    }
}
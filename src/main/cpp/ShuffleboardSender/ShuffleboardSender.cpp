#include "ShuffleboardSender/ShuffleboardSender.h"

ShuffleboardData::ShuffleboardData(std::string name){
    tab_ = &frc::Shuffleboard::GetTab(name);
}

void ShuffleboardData::add(ShuffleboardItem item){
    items_.push_back(item);
}

void ShuffleboardData::add(std::string name, double* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}
void ShuffleboardData::add(std::string name, bool* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}
void ShuffleboardData::add(std::string name, int* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}
void ShuffleboardData::add(std::string name, frc::PIDController* o, bool edit){
    items_.push_back(ShuffleboardItem(name, o, tab_, edit));
}

void ShuffleboardData::send(){
    int size = items_.size();
    for(int i = 0; i<size; i++){
        items_[i].send();
    }
};

void ShuffleboardData::edit(){
    int size = items_.size();
    for(int i = 0; i<size; i++){
        items_[i].edit();
    }
}
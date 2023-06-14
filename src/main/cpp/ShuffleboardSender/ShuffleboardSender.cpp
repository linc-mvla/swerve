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

template <typename T> void ShuffleboardSender::add(ShuffleboardItem<T> item){
    items_.push_back(&item);
}

template <typename T> void ShuffleboardSender::add(std::string name, T* o, bool edit){
    items_.push_back(ShuffleboardItem({name, tab_, edit}, o));
}

void ShuffleboardSender::update(){
    if(edit_){
        for(ShuffleboardItemInterface item : items_){
            item.edit();
        }
    }
    for(ShuffleboardItemInterface item : items_){
        item.send();
    }
};

#include "ShuffleboardSender/ShuffleboardItemDefined.h"
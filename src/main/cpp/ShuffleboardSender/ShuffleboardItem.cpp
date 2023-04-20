#include "ShuffleboardSender/ShuffleboardItem.h"

ShuffleboardItem::ShuffleboardItem(std::string name, double* o, frc::ShuffleboardTab* tab, bool edit):
    type_(DOUBLE)
{
    value_.d = o;
    entry_.one = tab->Add(name, *o).GetEntry();   
};

ShuffleboardItem::ShuffleboardItem(std::string name, bool* o, frc::ShuffleboardTab* tab, bool edit):
    type_(BOOL)
{
    value_.b = o;
    entry_.one = tab->Add(name, *o).GetEntry();   
};

ShuffleboardItem::ShuffleboardItem(std::string name, int* o, frc::ShuffleboardTab* tab, bool edit):
    type_(INT)
{
    value_.i = o;
    entry_.one = tab->Add(name, *o).GetEntry();   
};

ShuffleboardItem::ShuffleboardItem(std::string name, units::volt_t* o, frc::ShuffleboardTab* tab, bool edit):
    type_(VOLT)
{
    value_.volt = o;
    entry_.one = tab->Add(name, o->value()).GetEntry();   
};

ShuffleboardItem::ShuffleboardItem(std::string name, frc::PIDController* o, frc::ShuffleboardTab* tab, bool edit):
    type_(PIDCONTROLLER)
{
    value_.pid = o;
    entry_.pid[0] = tab->Add(name + " P", o->GetP()).GetEntry();   
    entry_.pid[1] = tab->Add(name + " I", o->GetI()).GetEntry();
    entry_.pid[2] = tab->Add(name + " D", o->GetD()).GetEntry();   
};

void ShuffleboardItem::send(){
    switch(type_){
        case DOUBLE:
            entry_.one->SetDouble(*value_.d);
            break;
        case BOOL:
            entry_.one->SetBoolean(*value_.b);
            break;
        case INT:
            entry_.one->SetInteger(*value_.i);
            break;
        case PIDCONTROLLER:
            entry_.pid[0]->SetDouble(value_.pid->GetP());
            entry_.pid[1]->SetDouble(value_.pid->GetI());
            entry_.pid[2]->SetDouble(value_.pid->GetD());
            break;
    }
}

void ShuffleboardItem::edit(){
    if(!edit_){
        return;
    }
    switch(type_){
        case DOUBLE:
            *value_.d = entry_.one->GetDouble(*value_.d);
            break;
        case BOOL:
            *value_.b = entry_.one->GetBoolean(*value_.b);
            break;
        case INT:
            *value_.i = entry_.one->GetInteger(*value_.i);
            break;
        case PIDCONTROLLER:
            value_.pid->SetP(entry_.pid[0]->GetDouble(value_.pid->GetP()));
            value_.pid->SetI(entry_.pid[1]->GetDouble(value_.pid->GetI()));
            value_.pid->SetD(entry_.pid[2]->GetDouble(value_.pid->GetD()));
            break;
    }
}
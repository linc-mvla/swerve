#include "ShuffleboardSender/ShuffleboardItem.h"

ShuffleboardItem::ShuffleboardItem(ItemData data, double* value):
    type_(DOUBLE),
    edit_(data.edit)
{
    value_.d = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entries_.push_back(data.tab->Add(data.name, *value)
                                            .WithSize(data.width, data.height)
                                            .WithPosition(data.positionX, data.positionY)
                                            .GetEntry()
                                        );
    } 
    else{
        entries_.push_back(data.tab->Add(data.name, *value).
                                            WithSize(data.width, data.height)
                                            .GetEntry());
    }
};

ShuffleboardItem::ShuffleboardItem(ItemData data, bool* value):
    type_(BOOL),
    edit_(data.edit)
{
    value_.b = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entries_.push_back(data.tab->Add(data.name, *value)
                                            .WithSize(data.width, data.height)
                                            .WithPosition(data.positionX, data.positionY)
                                            .GetEntry()
                                        );
    } 
    else{
        entries_.push_back(data.tab->Add(data.name, *value)
                                            .WithSize(data.width, data.height)
                                            .GetEntry());
    }
};

ShuffleboardItem::ShuffleboardItem(ItemData data, int* value):
    type_(INT),
    edit_(data.edit)
{
    value_.i = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entries_.push_back(data.tab->Add(data.name, *value)
                                            .WithSize(data.width, data.height)
                                            .WithPosition(data.positionX, data.positionY)
                                            .GetEntry()
                                        );
    } 
    else{
        entries_.push_back(data.tab->Add(data.name, *value)
                                            .WithSize(data.width, data.height)
                                            .GetEntry());
    }
};

ShuffleboardItem::ShuffleboardItem(ItemData data, units::volt_t* value):
    type_(VOLT),
    edit_(data.edit)
{
    value_.volt = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entries_.push_back(data.tab->Add(data.name, value->value())
                                            .WithSize(data.width, data.height)
                                            .WithPosition(data.positionX, data.positionY)
                                            .GetEntry()
                                        );
    } 
    else{
        entries_.push_back(data.tab->Add(data.name, value->value())
                                            .WithSize(data.width, data.height)
                                            .GetEntry());
    }
}

ShuffleboardItem::ShuffleboardItem(ItemData data, frc::PIDController* value):
    type_(PIDCONTROLLER),
    edit_(data.edit)
{
    value_.pid = value;

    frc::ShuffleboardLayout* pidLayout;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        pidLayout = &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                                    .WithSize(data.width, data.height)
                                                    .WithPosition(data.positionX, data.positionY)
                                                    .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
    }
    else{
        pidLayout = &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                                    .WithSize(data.width, data.height)
                                                    .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
    }
    entries_.push_back(pidLayout->Add("P", value->GetP()).GetEntry());   
    entries_.push_back(pidLayout->Add("I", value->GetI()).GetEntry());
    entries_.push_back(pidLayout->Add("D", value->GetD()).GetEntry());   
};

void ShuffleboardItem::send(){
    switch(type_){
        case DOUBLE:
            entries_[0]->SetDouble(*value_.d);
            break;
        case BOOL:
            entries_[0]->SetBoolean(*value_.b);
            break;
        case INT:
            entries_[0]->SetInteger(*value_.i);
            break;
        case PIDCONTROLLER:
            entries_[0]->SetDouble(value_.pid->GetP());
            entries_[1]->SetDouble(value_.pid->GetI());
            entries_[2]->SetDouble(value_.pid->GetD());
            break;
    }
}

void ShuffleboardItem::edit(){
    if(!edit_){
        return;
    }
    switch(type_){
        case DOUBLE:
            *value_.d = entries_[0]->GetDouble(*value_.d);
            break;
        case BOOL:
            *value_.b = entries_[0]->GetBoolean(*value_.b);
            break;
        case INT:
            *value_.i = entries_[0]->GetInteger(*value_.i);
            break;
        case PIDCONTROLLER:
            value_.pid->SetP(entries_[0]->GetDouble(value_.pid->GetP()));
            value_.pid->SetI(entries_[1]->GetDouble(value_.pid->GetI()));
            value_.pid->SetD(entries_[2]->GetDouble(value_.pid->GetD()));
            break;
    }
}
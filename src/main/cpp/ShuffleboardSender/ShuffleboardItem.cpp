#include "ShuffleboardSender/ShuffleboardItem.h"

template <typename T> ShuffleboardItem<T>::ShuffleboardItem(ItemData data, T* value):
    edit_(data.edit)
{
    value_ = value;
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

template <> ShuffleboardItem<units::volt_t>::ShuffleboardItem(ItemData data, units::volt_t* value):
    edit_(data.edit)
{
    value_ = value;
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

template <> ShuffleboardItem<frc::PIDController>::ShuffleboardItem(ItemData data, frc::PIDController* value):
    edit_(data.edit)
{
    value_ = value;

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

template <> void ShuffleboardItem<double>::send(){
    entries_[0]->SetDouble(*value_);
}
template <> void ShuffleboardItem<bool>::send(){
    entries_[0]->SetBoolean(*value_);
}
template <> void ShuffleboardItem<int>::send(){
    entries_[0]->SetInteger(*value_);
}
template <> void ShuffleboardItem<frc::PIDController>::send(){
    entries_[0]->SetDouble(value_->GetP());
    entries_[1]->SetDouble(value_->GetI());
    entries_[2]->SetDouble(value_->GetD());
}

template <> void ShuffleboardItem<double>::edit(){
    if(!edit_)return;
    *value_ = entries_[0]->GetDouble(*value_);
}
template <> void ShuffleboardItem<bool>::edit(){
    if(!edit_)return;
    *value_ = entries_[0]->GetBoolean(*value_);
}
template <> void ShuffleboardItem<int>::edit(){
    if(!edit_)return;
    *value_ = entries_[0]->GetInteger(*value_);
}
template <> void ShuffleboardItem<frc::PIDController>::edit(){
    if(!edit_)return;
    value_->SetP(entries_[0]->GetDouble(value_->GetP()));
    value_->SetI(entries_[1]->GetDouble(value_->GetI()));
    value_->SetD(entries_[2]->GetDouble(value_->GetD()));
}
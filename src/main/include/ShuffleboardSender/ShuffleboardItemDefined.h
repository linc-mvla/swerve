#pragma once

#include "ShuffleboardItem.h"
#include "SwerveDrive/SwervePose.h"

template<> class ShuffleboardItem<double> : public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, double* value);
        void send();
        void edit();
    private:
        bool edit_;
        double* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<bool>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, bool* value);
        void send();
        void edit();
    private:
        bool edit_;
        bool* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<int>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, int* value);
        void send();
        void edit();
    private:
        bool edit_;
        int* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<units::volt_t>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, units::volt_t* value);
        void send();
        void edit();
    private:
        bool edit_;
        units::volt_t* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<frc::PIDController>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, frc::PIDController* value);
        void send();
        void edit();
    private:
        bool edit_;
        frc::PIDController* value_;
        nt::GenericEntry* entry_[3]; //[P, I, D]
};

template<> class ShuffleboardItem<SwervePose::ModulePose>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, SwervePose::ModulePose* value);
        void send();
        void edit();
    private:
        bool edit_;
        SwervePose::ModulePose* value_;
        nt::GenericEntry* entry_[2]; //[Speed, Ang]
};

//template<> class ShuffleboardItem<SwervePose::Pose>;

#include "ShuffleboardItem.hpp"
#pragma once

#include <frc2/command/PIDCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/voltage.h>

#include <vector>

#include "SwerveDrive/SwervePose.h"

class ShuffleboardItemInterface{
    public:
        virtual void send();
        virtual void edit();
};

/**
 * Class to send objects to shuffleboard
 * 
 * note: edit "send" and "edit" when adding a type
*/
template <typename T>
class ShuffleboardItem: ShuffleboardItemInterface{
    public:
        /***
         * Structure to more easily create items
         * Also doesn't make the constructer ugly
        */
        struct ItemData{
            std::string name;
            frc::ShuffleboardTab* tab;
            bool edit = false;
            int width = 1;
            int height = 1;
            int positionX = -1;
            int positionY = -1;
        };
        ShuffleboardItem(ItemData data, T* value);
        // ShuffleboardItem<units::volt_t>(ItemData data, units::volt_t* value);
        // ShuffleboardItem<frc::PIDController>(ItemData data, frc::PIDController* value);
        // ShuffleboardItem<SwervePose::Pose>(ItemData data, SwervePose::Pose* value);
        // ShuffleboardItem<>(ItemData data, SwervePose::ModulePose* value);
        void send();
        void edit();

    private:
        bool edit_;

        T* value_;

        std::vector<nt::GenericEntry*> entries_;
};
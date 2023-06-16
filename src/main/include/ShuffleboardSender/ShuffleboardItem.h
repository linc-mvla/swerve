#pragma once

#include <vector>

#include <frc2/command/PIDCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/voltage.h>

#include "SwerveDrive/SwervePose.h"

std::vector<int> k;
class ShuffleboardItemInterface{
    public:
        struct ItemData{
            std::string name;
            frc::ShuffleboardTab* tab;
            bool edit = false;
            int width = 1;
            int height = 1;
            int positionX = -1;
            int positionY = -1;
        };
        virtual void send(){};
        virtual void edit(){};
};

/**
 * Class to send objects to shuffleboard
 * 
*/
template <typename T>
class ShuffleboardItem : public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, T* value);
        
        void send();
        void edit();

    private:
        bool edit_;

        T* value_;

        nt::GenericEntry* entry_;
};

#include "ShuffleboardItemDefined.h"
#include "ShuffleboardItem.hpp"
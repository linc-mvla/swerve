#pragma once

#include <frc2/command/PIDCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/voltage.h>

#include <vector>

#include "SwerveDrive/SwervePose.h"

/**
 * Class to send objects to shuffleboard
 * 
 * note: edit "send" and "edit" when adding a type
*/
class ShuffleboardItem{
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
        ShuffleboardItem(ItemData data, double* value);
        ShuffleboardItem(ItemData data, bool* value);
        ShuffleboardItem(ItemData data, int* value);
        ShuffleboardItem(ItemData data, units::volt_t* value);
        ShuffleboardItem(ItemData data, frc::PIDController* value);
        ShuffleboardItem(ItemData data, SwervePose::Pose* value);
        ShuffleboardItem(ItemData data, SwervePose::ModulePose* value);
        void send();
        void edit();

    private:
        bool edit_;

        enum type{ //Says what type the item is
            DOUBLE,
            BOOL,
            INT,
            VOLT,
            PIDCONTROLLER,
            SWERVEPOSE
        } type_;

        union{ //Pointer to the item's value to edit and read
            double *d;
            bool *b;
            int *i;
            units::volt_t *volt;
            frc::PIDController *pid; //Entries: [P, I, D]
            struct{
                frc::Field2d *field;
                SwervePose::Pose *pose;
            }swervePose;
            SwervePose::ModulePose *modulePose;
        } value_;

        std::vector<nt::GenericEntry*> entries_;
};
#pragma once

#include <frc2/command/PIDCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <units/voltage.h>

/**
 * Class to send objects to shuffleboard
 * 
 * note: edit "send" and "edit" when adding a type
*/
class ShuffleboardItem{
    public:
        ShuffleboardItem(std::string name, double* o,             frc::ShuffleboardTab* tab,  bool edit = false);
        ShuffleboardItem(std::string name, bool* o,               frc::ShuffleboardTab* tab,  bool edit = false);
        ShuffleboardItem(std::string name, int* o,                frc::ShuffleboardTab* tab,  bool edit = false);
        ShuffleboardItem(std::string name, units::volt_t* o,      frc::ShuffleboardTab* tab,  bool edit = false);
        ShuffleboardItem(std::string name, frc::PIDController* o, frc::ShuffleboardTab* tab,  bool edit = false);
        void send();
        void edit();

    private: 
        bool edit_;

        enum type{
            DOUBLE,
            BOOL,
            INT,
            VOLT,
            PIDCONTROLLER
        };

        type type_;

        union{
            double *d;
            bool *b;
            int *i;
            units::volt_t *volt;
            frc::PIDController *pid;
        } value_;

        union{
            nt::GenericEntry* one;
            nt::GenericEntry* pid[3]; //[P, I, D]
        } entry_;
};
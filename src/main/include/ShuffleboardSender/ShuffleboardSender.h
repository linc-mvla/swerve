#pragma once

#include <vector>
#include <string>

#include <frc/shuffleboard/Shuffleboard.h>

#include <units/voltage.h>

#include "ShuffleboardItem.h"
#include "SwerveDrive/SwervePose.h"

/**
 * Class to send many variables to Shuffleboard and edit them
*/
class ShuffleboardSender{
    public:
        /**
         * Creates a tab with name
        */
        ShuffleboardSender(std::string name);

        /**
         * Initializes the tab
        */
        void Initialize(bool edit = false);
        bool isInitialized(){return initialized_;}

        /**
         * Add a pointer to an variable to send/get
        */
        void add(ShuffleboardItem item);
        void add(std::string name, double* o, bool edit = false);
        void add(std::string name, bool* o, bool edit = false);
        void add(std::string name, int* o, bool edit = false);
        void add(std::string name, units::volt_t* o, bool edit=false);
        void add(std::string name, frc::PIDController* o, bool edit = false);
        void add(std::string name, SwervePose::Pose* o, bool edit = false);
        void add(std::string name, SwervePose::ModulePose* o, bool edit = false);
        
        /**
         * Sends the data from pointers
        */
        void send();
        /**
         * Updates variables
        */
        void edit();

    private:
        std::string name_;
        bool initialized_;
        bool edit_;
        frc::ShuffleboardTab* tab_;
        std::vector<ShuffleboardItem> items_;
};

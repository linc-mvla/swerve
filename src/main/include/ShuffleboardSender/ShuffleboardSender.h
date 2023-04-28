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
         * Updates variables by reading and configuring, and then sending the data
        */
        void update();

        /***
         * If the sender's edit is disabled, all items will not be able to be edited
        */
        void disable() {enabled_ = false;}
        void enable(bool edit = false) {enabled_ = true; edit_ = edit;}

    private:
        std::string name_;
        bool initialized_ = false;
        bool edit_ = false;
        bool enabled_ = false;
        frc::ShuffleboardTab* tab_;
        std::vector<ShuffleboardItem> items_;
};

#pragma once

#include <vector>
#include <string>

#include <frc/shuffleboard/Shuffleboard.h>

#include "ShuffleboardItem.h"

/**
 * Class to send many variables to Shuffleboard and edit them
*/
class ShuffleboardData{
    public:
        /**
         * Creates a tab with name
        */
        ShuffleboardData(std::string name);
        /**
         * Add a pointer to an variable to send/get
        */
        void add(ShuffleboardItem item);
        void add(std::string name, double* o, bool edit = false);
        void add(std::string name, bool* o, bool edit = false);
        void add(std::string name, int* o, bool edit = false);
        void add(std::string name, frc::PIDController* o, bool edit = false);
        /**
         * Sends the data from pointers
        */
        void send();
        /**
         * Updates variables
        */
        void edit();

    private:
        frc::ShuffleboardTab* tab_;
        std::vector<ShuffleboardItem> items_;
};

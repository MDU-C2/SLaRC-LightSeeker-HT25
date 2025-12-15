#pragma once

#include <string>
#include <array>

const int BITRATE = 1000000;

const std::array<int,2> LEFT_MOTORS = {1, 3};
const std::array<int,2> RIGHT_MOTORS = {2, 4};

const int CMD_SET_DUTY = 0;
const int CMD_SET_CURRENT = 1;
const int CMD_SET_BRAKE = 2;
const int CMD_SET_RPM = 3;
const int CMD_SET_POS = 4;
const int CMD_SET_ZERO = 5;
const int CMD_SET_POS_SPD = 6;

const int CMD_SET_MODE = 10;





// Motor
class Motors {
    
    public:
    Motors(std::string battery_IDm, std::string name);
    ~Motors();

    void pack_message_to_ROS(); //Returns msg -> BatteryInfo "struct"
    int update_from_frame(int data_from_buffer); //Translates data from CAN Bus hex format to decimal format
    bool allow_drive(); //Returns true if battery is OK

    private:
    int decode_error(); //Sets error flags with info from CAN bus

};

// MotorManager

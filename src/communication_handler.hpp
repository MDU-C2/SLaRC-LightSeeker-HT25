#pragma once

// Battery


const std::string LEFT_CAN_ID = "0x01109216";
const std::string RIGHT_CAN_ID = "0x01109217";


class Battery {
    
    public:
    Battery(std::string battery_IDm, std::string name);
    ~Battery();

    void pack_message_to_ROS(); //Returns msg -> BatteryInfo "struct"
    int update_from_frame(int data_from_buffer); //Translates data from CAN Bus hex format to decimal format
    bool allow_drive(); //Returns true if battery is OK

    private:
    int decode_error(); //Sets error flags with info from CAN bus

};

// BatteryManager
class BatteryManager {
    
    public:
    BatteryManager(std::string bus);
    ~BatteryManager();

    // Receives CAN frames, rebuilds multi-frame packets, and updates batteries
    int receive_packet(); //Uses Battery: update_from_frame

    private:
    void extract_tail(); //Extracts Start, End, Toggle, and Transfer-ID flags from the tail byte

    

};

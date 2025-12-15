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

class Motor {
    
    public:
    Motor(std::string bus, std::string motor_id);
    ~Motor();

    void set_left(int rpm); //Replace with additional argument in set_rpm?
    void set_right(int rpm); //Replace with additional argument in set_rpm?
    void set_mode(int mode);
    void set_rpm(int rpm);
    void stop();
    bool handle_rx();
    int pack_int32(int32_t value);// Moved from packers class in py version

    private:
    
};

class MotorGroup {

    public:
    MotorGroup(std::string left, std::string right, int track_width_m, int wheel_radius_m);
    ~MotorGroup();

    float twist_to_rpm(float v_x, float w_z);
    void apply_twist(float v_x, float w_z);
};

class MotorTelemetry {

    public:
    MotorTelemetry(int track_width_m);
    ~MotorTelemetry();

    int update_from_frame(int data);

};

class SkidSteerController {

    public:
    SkidSteerController(int group, int max_rpm, int turn_strength);
    ~SkidSteerController();

};

class MotorOdometry {

    public:
    MotorOdometry(int track_width_m);
    ~MotorOdometry();

    int update(float v_left, float v_right, float dt);

};

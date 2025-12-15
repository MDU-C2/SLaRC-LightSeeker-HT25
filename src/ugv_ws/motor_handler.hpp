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

class Motors {
    
    public:
    Motors(std::string bus, std::string motor_id);
    ~Motors();

    void set_mode(int mode);
    void set_rpm(int rpm);
    void stop();
    bool handle_rx();

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

class SkidSteerOdometry {

    public:
    SkidSteerOdometry(int track_width_m);
    ~SkidSteerOdometry();

    int update(float v_left, float v_right, float dt);

};

class packers {

    public:
    int pack_int32(int32_t value);
};

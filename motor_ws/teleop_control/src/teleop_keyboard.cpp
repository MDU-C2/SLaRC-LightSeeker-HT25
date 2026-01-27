#include <chrono>                   // Time Library 
#include <memory>                   // Manage dynamic memory 
#include <string>                   // String Library 
#include <cmath>
#include <SDL2/SDL.h>               // Reads Keyboard/mouse input 
#include <bits/stdc++.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <algorithm>

#pragma GCC diagnostic ignored "-Wswitch"

//Use this for name spaces std::
using namespace std::chrono_literals;


//Global variables
float Max_Linear = 1.0;
float Max_Angular = 2.0;
float Default_Linear = 0.05;
float Default_Angular = 0.50;
float Step_Size = 0.05;                //how fast speed changes per keyboard press/hold



//Make a simple class and inherining the class from the Node Class
class KeyBoardTeleop : public rclcpp::Node 
{
    public:
    // Constructor:
    // Call the parent constructor
    KeyBoardTeleop() : Node("teleop_keyboard")
    {
        //Initialize the objects 

        current_linear = Default_Linear;
        current_angular = Default_Angular;

        //Make a publish 
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 20);

        //Timer object 
        timer_ = this->create_wall_timer(20ms, std::bind(&KeyBoardTeleop::publish_velocity, this));

    }

    //Private: Only code inside the class can access this
    private:
    //Member variables
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;         //Store the publisher_
    rclcpp::TimerBase::SharedPtr timer_;

    //Robots current speed,
    float current_linear;
    float current_angular;

    // At the begining non keys are being hold 
    bool w_held = false;
    bool s_held = false;
    bool a_held = false;
    bool d_held = false;
    

    //SDL keeps list of events "thins that happend recently"
    //This function runs periodically (the timer calls is everyt Xms)
    // Checks if the window is closed 
    // Reads which keys are being held down 
    void publish_velocity()
    {
        SDL_Event e;

        //Keep going until there are no more events left in the queue
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT)
            {
                rclcpp::shutdown();
                return;
            }

            //If a key was pressed 
            if (e.type == SDL_KEYDOWN && !e.key.repeat)
            {
                //Check 
                switch (e.key.keysym.scancode)
                {
                    case SDL_SCANCODE_W: w_held = true; 
                        current_linear = Default_Linear; 
                        break;
                    case SDL_SCANCODE_S: s_held = true;
                        current_linear = Default_Linear; 
                        break;
                    case SDL_SCANCODE_A: a_held = true; 
                        current_angular = Default_Angular; 
                        break;
                    case SDL_SCANCODE_D: d_held = true; 
                        current_angular = Default_Angular; 
                        break;
                    
                    //We can only modify the speed if w,s,a,or d is being held    
                    case SDL_SCANCODE_UP:
                        if (w_held || s_held)
                            //It must not exceed the max value 
                            current_linear = std::min(current_linear + Step_Size, Max_Linear);
                        if (a_held || d_held)
                            current_angular = std::min(current_angular + Step_Size*(2), Max_Angular);
                        break;

                    case SDL_SCANCODE_DOWN:
                        if (w_held || s_held)
                            current_linear = std::max(current_linear - Step_Size, -0.1f);
                        if (a_held || d_held)
                            current_angular = std::max(current_angular - Step_Size*(2), 0.0f);
                        break;
                    default:
                        break;
                }
            }

            //Check if the key is released
            if (e.type == SDL_KEYUP)
            {
                switch (e.key.keysym.scancode)
                {
                    case SDL_SCANCODE_W: 
                        w_held = false; 
                        current_linear = Default_Linear; 
                        break;
                    case SDL_SCANCODE_S: 
                        s_held = false; 
                        current_linear = Default_Linear; 
                        break;
                    case SDL_SCANCODE_A: 
                        a_held = false; 
                        current_angular = Default_Angular; 
                        break;
                    case SDL_SCANCODE_D: 
                        d_held = false; 
                        current_angular = Default_Angular; 
                        break;
                }
            }
        }

        geometry_msgs::msg::TwistStamped velocity;
        velocity.header.frame_id = "base_link";
        velocity.header.stamp = this->get_clock()->now();
        velocity.twist.linear.x = 0.0;
        velocity.twist.angular.z = 0.0;

        //Publish the speed while the wasd keys are being hold
        // +- = add/remove something from the existing value
        if (w_held) velocity.twist.linear.x += current_linear;
        if (s_held) velocity.twist.linear.x -= current_linear;
        if (a_held) velocity.twist.angular.z += current_angular;
        if (d_held) velocity.twist.angular.z -= current_angular;

        RCLCPP_INFO(this->get_logger(), "linear.x=%.2f angular.z=%.2f (lin=%.2f ang=%.2f)",
                    velocity.twist.linear.x, velocity.twist.angular.z, current_linear, current_angular);

        publisher_->publish(velocity);
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<KeyBoardTeleop> node = std::make_shared<KeyBoardTeleop>();

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0)
    {
        std::cerr << "SDL_Init error: " << SDL_GetError() << "\n";
        return 1;
    }

    // We need a window (can be hidden) so SDL can receive keyboard focus.
    SDL_Window *win = SDL_CreateWindow("Teleop Window",SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,640, 480, SDL_WINDOW_SHOWN);
    if (!win)
    {
        std::cerr << "SDL_CreateWindow error: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    rclcpp::spin(node);

    SDL_DestroyWindow(win);
    SDL_Quit();
    rclcpp::shutdown();
    return 0;
}
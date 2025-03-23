#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <stdlib.h>
#include <chrono>
#include <map>
#include <deque>
#include <numeric>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <thread>

class CubemarsControl {
    public:
        CubemarsControl();
        CubemarsControl(int motor_id, int socket);
        //Turret Motor Variables


        struct Cubemars_Motor {
            int motor_id;
            float position;
            float speed;
            float torque;
            float temperature;
            int error_flag;
        };

        struct Cubemars_Input{
            float pos;
            float vel;
            float kp;
            float kd;
            float torq;
        };

        int float_to_uint(float x, float x_min, float x_max, int bits);
        void sendCommandMITMode(float pos, float vel, float kp, float kd, float torq);
        void enterMITMode();
        void exitMITMode();
        void zeroMotor();
        float uint_to_float(int x_int, float x_min, float x_max, int bits);
        int getMotorID();
        Cubemars_Motor& getMotorData();
        //Cubemars_Motor unpackReply();
        bool unpackReply();
        bool waitForReply(int timeout_usecs);
        void toc(std::string word);
        std::chrono::time_point<std::chrono::high_resolution_clock> getTimeOfLastRead();

    private:
        int kDefaultTimeoutUsecs = 10000; //usecs
        int kZeroMotorTimeoutUsecs = 2000000; //usecs

        std::chrono::time_point<std::chrono::high_resolution_clock> clock_;
        std::chrono::time_point<std::chrono::high_resolution_clock> time_of_last_read_;

        float P_MIN=-12.5f;
        float P_MAX=12.5f;
        float V_MIN=-50.0f;
        float V_MAX=50.0f;
        float T_MIN=-65.0f;
        float T_MAX=65.0f;
        float KP_MIN=0;
        float KP_MAX=500.0f;
        float KD_MIN=0;
        float KD_MAX=5.0f;

        Cubemars_Motor motor_data_;
        int motor_id_;
        int sock_;
        int nbytes_;

};
#endif
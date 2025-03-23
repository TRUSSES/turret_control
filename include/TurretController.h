#pragma once

#include <iostream>
#include <cmath>
#include <cstring>
#include <pigpio.h>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>

#include <map>
#include <deque>
#include <numeric>
#include <vector>
#include <atomic>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <thread>

#include "cubemars_control.h"

using namespace std;

class TurretController{

    private:
        bool in_positive_hysterisis_ = false;
        bool in_negative_hysterisis_ = false;
		// gpio pins

        // Spiral Zipper encoder pin mapping 
		unsigned int SZ_ENC_CS_PIN = 13;
        unsigned int SZ_ENC_CLK_PIN = 26;
        unsigned int SZ_ENC_DO_PIN = 19;

        // spiral zipper encoder variables
        int _sz_encoder_count = 0;
        int _sz_prev_encoder_value = 0;
        double EXTENSION_PER_STEP = 0.000004453125; 
        int ENCODER_MAX_VALUE = 1023;

        // Turret pitch encoder pin mapping 
        unsigned int TUR_ENC_CS_PIN = 0;
        unsigned int TUR_ENC_CLK_PIN = 6;
        unsigned int TUR_ENC_DO_PIN = 5;

        // Turret pitch encoder variables
        int _turret_encoder_count = 0;
        int _turret_prev_encoder_value = 0;
        float _turret_offset = 0;
        

        // ESC (electronic speed controller) pin mapping and variables
        int _sz_esc_gpio_pin = 22;
		int _sz_CCW_PWM = 1000;
		int _sz_CC_PWM = 2000;
		int _sz_STOP_PWM = 1500;

		int _sz_PWM = 0;
		long _sz_time = 0;
		uint32_t _sz_start_time;
		uint32_t _sz_end_time;
        int _sz_des_pos = 0;

        int dir_ = 0;
		

        // Limit switch Variables and pin mapping
        int _sz_limit_switch = 21;
        int _turret_limit_switch = 20;
        
        bool _sz_limit_switch_last_state = false;
        bool _turret_limit_switch_last_state = false;
    	std::chrono::steady_clock::time_point sz_lastDebounceTime;
        std::chrono::steady_clock::time_point turret_lastDebounceTime;

        bool _sz_limit_switch_pressed = false;
        bool _turret_limit_switch_pressed = false;

        static TurretController* instance;

        // Load Cell pin mapping and variables
        int _load_cell_sck = 1;
        int _load_cell_1_dt = 25; 
        int _load_cell_2_dt = 23;
        int _load_cell_3_dt = 24;

        float loadcell_calibrationValue = 696.0;  // Set the calibration value (adjust based on your calibration)
        int loadcell_t = 0;

        //Turret Motor Variables
        
        CubemarsControl yaw_motor;
        float kp_ = 0;
        float kd_ = 0;
        float ki_ = 0;
        int sock_;
        int nbytes_;
        int ret_;
        struct sockaddr_can addr_;
        struct ifreq ifr_;
    
        int pitch_motor_id = 0xA;
        int yaw_motor_id = 0xB;

        // pitch control variables
        float y_offset = 0; // meters
        float x_offset = 0; // meters
        
    
    public:
    
        std::thread load_cell_thread_;
        CubemarsControl pitch_motor;
        TurretController();
        void initTurret();
        void zeroZipper();
        void zeroTurret();
        int openSocket();
        int closeSocket();

        static void checkZipperLimitSwitchStatic(int gpio, int level, uint32_t tick);
        static void checkTurretLimitSwitchStatic(int gpio, int level, uint32_t tick);

        void checkZipperLimitSwitch(int gpio, int level, uint32_t tick);
        void checkTurretLimitSwitch(int gpio, int level, uint32_t tick);

        int readSZEncoder();
        int readTurretEncoder();
        void updateSzEncoderCount();
        void updateTurretEncoderCount();

        void actuateZipperLength(float goal_dist);
        void stopMotor();
		int rampTrajectory(int goal_pos, int pwm, int des_pos, float dt);
		int pController(int des_pos, int curr_pos);
		void setMotorOutput(int pulse_width);
        float getActuatorLength();
        float getTurretAngle();

        bool actuateTurret(float zipper_length, float pitch, float yaw);
        bool actuateTurretCable(float goal_dist, float pitch, float yaw);

        std::atomic<int32_t> loadCell1{0};
        std::atomic<int32_t> loadCell2{0};
        std::atomic<int32_t> loadCell3{0};

        int32_t getLoadCell1Val() const;
        int32_t getLoadCell2Val() const;
        int32_t getLoadCell3Val() const;
        
        void readLoadCells();
        int32_t readRawHX711(int dtPin);
        void initializeHX711(int dtPin);

        // Turret pitch and yaw motor values
        


};

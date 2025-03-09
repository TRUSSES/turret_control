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
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <rclcpp/rclcpp.hpp>

#include <thread>

using namespace std;

class TurretController{

    private:
        bool in_positive_hysterisis_ = false;
        bool in_negative_hysterisis_ = false;
		// gpio pins

        // Spiral Zipper encoder pin mapping 
		unsigned int SZ_ENC_CS_PIN = 13;
        unsigned int SZ_ENC_CLK_PIN = 26;
        //const unsigned int SZ_ENC_DI_PIN = 18;
        unsigned int SZ_ENC_DO_PIN = 19;

        // spiral zipper encoder variables
        int _sz_encoder_count = 0;
        int _prev_encoder_value = 0;
        int _prev_avg_encoder_value = 0;
        double EXTENSION_PER_STEP = 0.000004453125; 
        int ENCODER_MAX_VALUE = 1023;

        // TODO - Turret pitch encoder pin mapping 
        unsigned int TUR_ENC_CS_PIN = 0;
        //const unsigned int PITCH_ENC_DI_PIN = 18;
        unsigned int TUR_ENC_CLK_PIN = 6;
        unsigned int TUR_ENC_DO_PIN = 5;

        // Turret pitch encoder variables
        int _turret_encoder_count = 0;
        

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
        float P_MIN;
        float P_MAX;
        float V_MIN;
        float V_MAX;
        float KP_MIN;
        float KP_MAX;
        float KD_MIN;
        float KD_MAX;
        float T_MIN;
        float T_MAX;

        int sock;
        int nbytes;
        int ret;
        struct sockaddr_can addr;
        struct ifreq ifr;
    
    
    public:
        
        std::thread load_cell_thread_;

        TurretController();
        void initTurret();
        void zeroZipper();

        static void checkZipperLimitSwitchStatic(int gpio, int level, uint32_t tick);
        static void checkTurretLimitSwitchStatic(int gpio, int level, uint32_t tick);

        void checkZipperLimitSwitch(int gpio, int level, uint32_t tick);
        void checkTurretLimitSwitch(int gpio, int level, uint32_t tick);

        int readSZEncoder();
        int readTurretEncoder();
        void updateEncoderCount();

        void actuateZipperLength(float goal_dist);
        void stopMotor();
		int rampTrajectory(int goal_pos, int pwm, int des_pos, float dt);
		int pController(int des_pos, int curr_pos);
		void setMotorOutput(int pulse_width);
        float getActuatorLength();

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
        int pitch_motor_id = 0xa;
        int yaw_motor_id = 0xb;



        struct Cubemars_Motor {
            int motor_id;
            float position;
            float speed;
            float current;
            float torque;
            float temperature; 
            int error_flag;
        };

        int float_to_uint(float x, float x_min, float x_max, int bits);
        void sendToTurretMotor(int motor_ID, float pos, float vel, float kp, float kd, float torq);
        void activateTurretMotors(int motor_ID);
        void deactivateTurretMotors(int motor_ID);
        Cubemars_Motor readTurretMotorValues(int motor_id);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);
        void closeSocket();

};

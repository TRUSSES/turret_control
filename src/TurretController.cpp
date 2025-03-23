#include "TurretController.h"

TurretController* TurretController::instance = nullptr;


TurretController::TurretController(){
    cout << "Starting Turret Controller \n";
	
    if (gpioInitialise() <0){
		cerr << "Error initializing pigpio. \n";
		exit(0);
	}

	// setting GPIO modes as input/output
	gpioSetMode(_sz_esc_gpio_pin, PI_OUTPUT);
	gpioSetMode(_sz_limit_switch, PI_INPUT);
	gpioSetMode(_turret_limit_switch, PI_INPUT);
	gpioSetMode(SZ_ENC_CS_PIN, PI_OUTPUT);
    gpioSetMode(SZ_ENC_CLK_PIN, PI_OUTPUT);
    gpioSetMode(SZ_ENC_DO_PIN, PI_INPUT);
	gpioSetMode(TUR_ENC_CS_PIN, PI_OUTPUT);
    gpioSetMode(TUR_ENC_CLK_PIN, PI_OUTPUT);
    gpioSetMode(TUR_ENC_DO_PIN, PI_INPUT);

    gpioWrite(SZ_ENC_CS_PIN, PI_HIGH); // Deselect the chip
    gpioWrite(SZ_ENC_CLK_PIN, PI_LOW); // Start with the clock low

	gpioWrite(TUR_ENC_CS_PIN, PI_HIGH); // Deselect the chip
    gpioWrite(TUR_ENC_CLK_PIN, PI_LOW); // Start with the clock low

	// set up pull up resistor on GPIO pin
	gpioSetPullUpDown(_sz_limit_switch, PI_PUD_UP);
	gpioSetPullUpDown(_turret_limit_switch, PI_PUD_UP);

	sz_lastDebounceTime = std::chrono::steady_clock::now();
	turret_lastDebounceTime = std::chrono::steady_clock::now();

}

int TurretController::openSocket() {
    // 1.Create socket
    //RCLCPP_INFO(logger_, "Opening Socket...");
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
    {
        perror("socket PF_CAN failed");
        exit(0);
        return 1;
    }

    // 2.Specify can0 device
    strcpy(ifr_.ifr_name, "can0");
    ret_ = ioctl(sock_, SIOCGIFINDEX, &ifr_);
    if (ret_ < 0)
    {
        perror("ioctl failed");
        exit(0);
        return 1;
    }

    // 3.Bind the socket to can0
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    ret_ = bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_));
    if (ret_ < 0)
    {
        perror("bind failed");
        exit(0);
        return 1;
    }
    //RCLCPP_INFO(logger_, "Opened Socket");
    return 0;
}

int TurretController::closeSocket() {
    //RCLCPP_INFO(logger_, "Closing Socket...");
    close(sock_);
    //RCLCPP_INFO(logger_, "Closed Socket");
    return 0;
}

void TurretController::initTurret(){
	cout << "Intitalizing ESC -> ";
	instance = this;
	gpioServo(_sz_esc_gpio_pin, _sz_STOP_PWM);
	sleep(2);
    
	// register alert functions to detect changes in gpio pin readings
    _sz_limit_switch_pressed = gpioRead(_sz_limit_switch);
    _turret_limit_switch_pressed = gpioRead(_turret_limit_switch);
    _sz_limit_switch_last_state = _sz_limit_switch_pressed;
    _turret_limit_switch_last_state = _turret_limit_switch_pressed; 

    gpioSetAlertFunc(_sz_limit_switch, checkZipperLimitSwitchStatic);
	gpioSetAlertFunc(_turret_limit_switch, checkTurretLimitSwitchStatic);

	//Load cell initialization 
	initializeHX711(_load_cell_1_dt);
    initializeHX711(_load_cell_2_dt);
    initializeHX711(_load_cell_3_dt);

    sleep(1);

    load_cell_thread_ = std::thread(&TurretController::readLoadCells,this);

    openSocket();

    pitch_motor = CubemarsControl(pitch_motor_id, sock_);
    //yaw_motor = CubemarsControl(yaw_motor_id, sock_);

    pitch_motor.enterMITMode();
    //yaw_motor.enterMITMode();

    cout << "DONE\n";
}

void TurretController::zeroZipper(){
	while (_sz_limit_switch_pressed == false) {
		gpioServo(_sz_esc_gpio_pin, 1650);
	}
	gpioServo(_sz_esc_gpio_pin, _sz_STOP_PWM);
	
	_sz_encoder_count = 0;
}

void TurretController::zeroTurret() {
    cout << "Starting zero turret" << endl;
    // Duration both limit switches must be continuously pressed before finalizing zeroing.
    const int zeroing_timeout_ms = 200;
    auto bothPressedStart = std::chrono::steady_clock::now();
    bool timerStarted = false;

    while (true) {
        // Get the current limit switch states.
        // Note: These variables are updated via the GPIO alert callbacks.
        bool zipperPressed = _sz_limit_switch_pressed;    // True when spiral zipper limit reached.
        bool turretPressed = _turret_limit_switch_pressed;  // True when turret pitch limit reached.

        // --- Spiral Zipper (Cable) Zeroing ---
        // When not yet at the limit, command a fixed retraction speed.
        if (!zipperPressed) {
            // Command a constant retract PWM.
            // (1650 was used in the original code to retract.)
            setMotorOutput(1650);
        } else {
            // Once the limit switch is hit, immediately stop the motor.
            setMotorOutput(_sz_STOP_PWM);
        }

        // --- Turret Pitch Zeroing ---
        // Command a fixed negative velocity until the limit switch is engaged.
        if (!turretPressed) {
            // Command a constant velocity (e.g. -3) toward the zero position.
            pitch_motor.sendCommandMITMode(0, -3, 0, 1, 0);
        } else {
            // When the turret limit switch is active, command zero velocity.
            pitch_motor.sendCommandMITMode(0, 0, 0, 0, 0);
        }

        // --- Check for Continuous Zeroing ---
        // Only start timing when both limit switches are engaged.
        if (zipperPressed && turretPressed) {
            if (!timerStarted) {
                bothPressedStart = std::chrono::steady_clock::now();
                timerStarted = true;
            }
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - bothPressedStart).count();
            if (elapsed_ms >= zeroing_timeout_ms) {
                // Finalize zeroing: record offsets and reset encoder counts.
                int raw_turret = readTurretEncoder();
                _turret_offset = raw_turret;
                _turret_encoder_count = 0;
                _turret_prev_encoder_value = 0;
                _sz_encoder_count = 0;

                // Stop both motors.
                pitch_motor.sendCommandMITMode(0, 0, 0, 0, 0);
                setMotorOutput(_sz_STOP_PWM);

                cout << "Zero Turret Done!" << endl;
                break;
            }
        } else {
            // If either limit is not engaged, reset the timer.
            timerStarted = false;
        }

        // Small delay to prevent hogging the CPU and allow limit switch updates.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


// Static callback function that redirects to the instance-specific handler
void TurretController::checkZipperLimitSwitchStatic(int gpio, int level, uint32_t tick) {
    if (instance) {
        instance->checkZipperLimitSwitch(gpio, level, tick); // Redirect to instance method
    }
}

void TurretController::checkTurretLimitSwitchStatic(int gpio, int level, uint32_t tick) {
    if (instance) {
        instance->checkTurretLimitSwitch(gpio, level, tick); // Redirect to instance method
    }
}


void TurretController::checkZipperLimitSwitch(int gpio, int level, uint32_t tick) {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - sz_lastDebounceTime);

    if (duration.count() > 200) {
        bool currentState = level;
        if (currentState != _sz_limit_switch_last_state) {
            if (currentState == 1) {
				//_sz_encoder_count = 0; Commented out to prevent resetting the encoder count during accidental limit switch press
				setMotorOutput(_sz_STOP_PWM);
				_sz_limit_switch_pressed = true;
                std::cout << "Spiral Zipper Limit Switch pressed" << std::endl;
            } else {
				_sz_limit_switch_pressed = false;
                std::cout << "Spiral Zipper Limit Switch released" << std::endl;
            }
            _sz_limit_switch_last_state = currentState;
        }
        sz_lastDebounceTime = now;
    }
}

void TurretController::checkTurretLimitSwitch(int gpio, int level, uint32_t tick) {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - turret_lastDebounceTime);

    if (duration.count() > 200) {
        bool currentState = level;
        if (currentState != _turret_limit_switch_last_state) {
            if (currentState == 1) {
				_turret_encoder_count = 0;
				// TODO : Stop the Pitch Motor
				_turret_limit_switch_pressed = true;
                std::cout << "Turret Pitch Limit Switch pressed" << std::endl;
            } else {
				_turret_limit_switch_pressed = false;
                std::cout << "Turret Pitch Limit Switch released" << std::endl;
            }
            _turret_limit_switch_last_state = currentState;
        }
        turret_lastDebounceTime = now;
    }
}

void TurretController::setMotorOutput(int pulse_width) {
    gpioServo(_sz_esc_gpio_pin, pulse_width); // Set the PWM pulse width
}

int TurretController::readSZEncoder() {
    static int last_value = 0;  // To store the last encoder reading
    uint16_t value = 0;
    
    // Select the encoder chip
    gpioWrite(SZ_ENC_CS_PIN, PI_LOW);
    gpioDelay(5); // Small delay to let the chip select stabilize

    // Read 16 bits from the encoder
    for (int i = 0; i < 16; i++) {
        gpioWrite(SZ_ENC_CLK_PIN, PI_HIGH);
        gpioDelay(5); // Adjust the delay as needed
        value <<= 1;
        if (gpioRead(SZ_ENC_DO_PIN)) {
            value |= 1;
        }
        gpioWrite(SZ_ENC_CLK_PIN, PI_LOW);
        gpioDelay(5); // Adjust the delay as needed
    }

    // Deselect the encoder chip
    gpioWrite(SZ_ENC_CS_PIN, PI_HIGH);

    // Shift off the first 6 bits to get the 10-bit position value
    value >>= 6;

    // Check if the value is in the expected 10-bit range (0-1023)
    if (value > 1023) {
        std::cerr << "Warning: Encoder value out of range: " << value << std::endl;
        value &= 0x03FF; // Mask to keep it within 10 bits
    }

    return value;
}

void TurretController::updateSzEncoderCount() {
    int current_encoder_value = readSZEncoder();

	 // Calculate the difference between the current and previous encoder values
    int difference = current_encoder_value - _sz_prev_encoder_value;

    // Handle wrap-around based on the direction of movement
    if (difference > ENCODER_MAX_VALUE / 2) {
        // Positive wrap-around (e.g., from 1023 to 0)
        difference -= (ENCODER_MAX_VALUE + 1);
    } else if (difference < -ENCODER_MAX_VALUE / 2) {
        // Negative wrap-around (e.g., from 0 to 1023)
        difference += (ENCODER_MAX_VALUE + 1);
    }

    // Update continuous encoder count
    _sz_encoder_count -= difference;

    // Update previous encoder value for the next calculation
    _sz_prev_encoder_value = current_encoder_value;

}


int TurretController::readTurretEncoder() {
    uint16_t value = 0;
    
    // Select the encoder chip
    gpioWrite(TUR_ENC_CS_PIN, PI_LOW);
    gpioDelay(5); // Small delay to let the chip select stabilize

    // Read 16 bits from the encoder
    for (int i = 0; i < 16; i++) {
        gpioWrite(TUR_ENC_CLK_PIN, PI_HIGH);
        gpioDelay(5); // Adjust the delay as needed
        value <<= 1;
        if (gpioRead(TUR_ENC_DO_PIN)) {
            value |= 1;
        }
        gpioWrite(TUR_ENC_CLK_PIN, PI_LOW);
        gpioDelay(5); // Adjust the delay as needed
    }

    // Deselect the encoder chip
    gpioWrite(TUR_ENC_CS_PIN, PI_HIGH);

    // Shift off the first 6 bits to get the 10-bit position value
    value >>= 6;

    // Check if the value is in the expected 10-bit range (0-1023)
    if (value > 1023) {
        std::cerr << "Warning: Encoder value out of range: " << value << std::endl;
        value &= 0x03FF; // Mask to keep it within 10 bits
    }

    return value;
}

void TurretController::updateTurretEncoderCount() {
    int raw_encoder_value = readTurretEncoder();

    // Compute encoder value relative to the stored zero point
    int current_encoder_value = raw_encoder_value - _turret_offset;
    
    // Handle wrap-around correction
    if (current_encoder_value < 0) {
        current_encoder_value += ENCODER_MAX_VALUE + 1;  // Ensure it's within [0, 1023]
    } else if (current_encoder_value > ENCODER_MAX_VALUE) {
        current_encoder_value -= ENCODER_MAX_VALUE + 1;
    }

    int difference = current_encoder_value - _turret_prev_encoder_value;

    // Wrap-around correction
    if (difference > ENCODER_MAX_VALUE / 2) {
        difference -= (ENCODER_MAX_VALUE + 1);
    } else if (difference < -ENCODER_MAX_VALUE / 2) {
        difference += (ENCODER_MAX_VALUE + 1);
    }

    _turret_encoder_count -= difference;
    _turret_prev_encoder_value = current_encoder_value;
}


float TurretController::getActuatorLength(){
    return _sz_encoder_count * (EXTENSION_PER_STEP * 4);
}

int TurretController::rampTrajectory(int goal_pos, int pwm, int des_pos, float dt){
	float max_step = (abs)(pwm-1500) * dt;
	float diff = goal_pos - des_pos;

	if (abs(diff) <= max_step){
		des_pos = goal_pos;
	} 
	else {
		if (diff > 0) {
		des_pos += max_step;
		} else {
		des_pos -= max_step;
		}
	}
	return des_pos;
}

int TurretController::pController(int des_pos, int curr_pos){
	int kp = 0.15;
	int err = des_pos - curr_pos;
	int control_action;
	_sz_PWM = err * kp + 1500;
    int deadband = 200;
	if(err > deadband/2){
		control_action = std::max (1350, std::min(_sz_CCW_PWM, _sz_PWM));
        dir_=-1;
	}else if(err < -deadband/2){
		control_action = std::max (1650, std::min(_sz_CC_PWM, _sz_PWM));
        dir_=1;
    }
    else{
        if(dir_==-1){
            if(err>0){
                control_action = std::max (1350, std::min(_sz_CCW_PWM, _sz_PWM));
            }else{
                dir_=0;
            }
        }
        //
        if(dir_==1){
            if(err<0){
                control_action = std::max (1650, std::min(_sz_CC_PWM, _sz_PWM));
            }else{
                dir_=0;
            }
        }
        if(dir_==0){
            control_action = 1500;
        }
        // control_action=1500;
    }

	return control_action;
}

void TurretController::actuateZipperLength(float goal_dist) {

    // Convert the goal distance to the corresponding encoder count
    int goal_count = static_cast<int>(goal_dist / EXTENSION_PER_STEP/4);

    //int _sz_PWM = (goal_count - _sz_encoder_count < 0) ? 1350 : 1650;

    updateSzEncoderCount();
    _sz_PWM = pController(goal_count, _sz_encoder_count);

    if (_sz_limit_switch_pressed && _sz_encoder_count > goal_count) {
        setMotorOutput(1500); // Stop motor
    }else{
        setMotorOutput(_sz_PWM);
    }

    return;

}

void TurretController::stopMotor(){
    setMotorOutput(1500);
}

void TurretController::readLoadCells()
{
    while(true){
        int32_t temp_val1 = readRawHX711(_load_cell_1_dt);
        int32_t temp_val2 = readRawHX711(_load_cell_2_dt);
        int32_t temp_val3 = readRawHX711(_load_cell_3_dt);
        loadCell1.store(temp_val1, std::memory_order_relaxed);
        loadCell2.store(temp_val2, std::memory_order_relaxed);
        loadCell3.store(temp_val3, std::memory_order_relaxed);
    }
}

void TurretController::initializeHX711(int dtPin)
{
    gpioSetMode(dtPin, PI_INPUT);           // Set DT pin as input
    gpioSetMode(_load_cell_sck, PI_OUTPUT); // Set shared SCK pin as output
    gpioWrite(_load_cell_sck, PI_LOW);      // Ensure SCK starts low
}

// Function to read raw data from HX711
int32_t TurretController::readRawHX711(int dtPin)
{
    // Wait for data ready
    int attempts = 0;
    while (gpioRead(dtPin) == PI_HIGH && attempts < 100000)
    {
        // Busy wait
        gpioDelay(1);
        attempts++;
    }

    int32_t value = 0;

    for (int i = 0; i < 24; i++)
    {
        gpioWrite(_load_cell_sck, PI_HIGH);
        gpioDelay(1);
        value = (value << 1) | gpioRead(dtPin);
        gpioWrite(_load_cell_sck, PI_LOW);
        gpioDelay(1);
    }
    // Additional pulse for gain
    gpioWrite(_load_cell_sck, PI_HIGH);
    gpioDelay(1);
    gpioWrite(_load_cell_sck, PI_LOW);
    gpioDelay(1);

    // Perform sign extension if the sign bit is set
    if (value & 0x800000)
    {
        value |= 0xFF000000;
    }
    
    return value;
}

int32_t TurretController::getLoadCell1Val() const {
    return loadCell1.load(std::memory_order_relaxed);
}

int32_t TurretController::getLoadCell2Val() const {
    return loadCell2.load(std::memory_order_relaxed);
}

int32_t TurretController::getLoadCell3Val() const {
    return loadCell3.load(std::memory_order_relaxed);
}

float TurretController::getTurretAngle(){
    return (_turret_encoder_count * 360)/1024;
}

bool TurretController::actuateTurret(float zipper_length, float desired_pitch, float yaw) {
    // Update the extension (zipper) actuator first.
    int goal_count = static_cast<int>(zipper_length / EXTENSION_PER_STEP / 4);
    updateSzEncoderCount();
    updateTurretEncoderCount();
    _sz_PWM = pController(goal_count, _sz_encoder_count);

    if (_sz_limit_switch_pressed && _sz_encoder_count > goal_count) {
         setMotorOutput(1500); // Stop zipper motor if limit is reached.
    } else {
         setMotorOutput(_sz_PWM);
    }

    // --- Turret Pitch Control ---
    // Get the current turret pitch angle.
    float current_pitch = getTurretAngle();
    float error = desired_pitch - current_pitch;
    
    // Define a tolerance (deadband) to avoid chasing small errors.
    const float pitch_tolerance = 3.0;  // degrees; adjust as needed

    // Proportional gain for the turret control. Tuning this value is key.
    const float k_p = 0.5;  

    // Compute the velocity command using proportional control.
    float velocity_command = - k_p * error;
    
    // Clamp the velocity command to the range [-3, 3] to avoid too aggressive corrections.
    if (velocity_command > 3.0) {
        velocity_command = 3.0;
    } else if (velocity_command < -3.0) {
        velocity_command = -3.0;
    }
    
    // If the error is within the deadband, command zero velocity.
    if (fabs(error) < pitch_tolerance) {
        velocity_command = 0.0;
    }
    
    // Send the command to the turret motor. (Other parameters remain unchanged.)
    // Note: Adjust sign if your motor coordinate system requires it.
    pitch_motor.sendCommandMITMode(0, velocity_command, 0, 1.5, 0);

    const int zipper_tolerance = 100;  // Adjust based on resolution and noise.
    bool zipperReached = (abs(goal_count - _sz_encoder_count) <= zipper_tolerance);
    bool pitchReached = (fabs(error) < pitch_tolerance);
    
    //cout << "zipper goal reached : " << zipperReached << endl;
    //cout << "pitch goal reached : " << pitchReached << endl;
    // Return true only if both the zipper length and turret pitch are within tolerance.
    return (zipperReached && pitchReached);
}

// bool TurretController::actuateTurretCable(float goal_dist, float pitch, float yaw) {
//     // --- Step 1: Update encoder counts and control the spiral zipper motor ---
//     int goal_count = static_cast<int>(goal_dist / EXTENSION_PER_STEP / 4);
//     updateSzEncoderCount();
//     updateTurretEncoderCount();

//     _sz_PWM = pController(goal_count, _sz_encoder_count);
//     if (_sz_limit_switch_pressed && _sz_encoder_count > goal_count) {
//          setMotorOutput(1500); // Stop zipper motor if limit reached.
//     } else {
//          setMotorOutput(_sz_PWM);
//     }

//     // --- Step 2: Compute current cable length based on actuator extension and turret angle ---
//     float current_L = getActuatorLength();  
//     float measured_pitch = getTurretAngle();  // in degrees
//     float current_r = x_offset + current_L * cos(measured_pitch * M_PI / 180.0);
//     float current_theta = atan((y_offset - current_L * sin(measured_pitch * M_PI / 180.0)) /
//                                (x_offset + current_L * cos(measured_pitch * M_PI / 180.0)));
//     float current_cable_length = current_r * current_theta;

//     // --- Step 3: Compute desired cable length from goal_dist and desired pitch ---
//     float desired_r = x_offset + goal_dist * cos(pitch * M_PI / 180.0);
//     float desired_theta = atan((y_offset - goal_dist * sin(pitch * M_PI / 180.0)) /
//                                (x_offset + goal_dist * cos(pitch * M_PI / 180.0)));
//     float desired_cable_length = desired_r * desired_theta;

//     // --- Step 4: Compute velocity command with feedforward + feedback ---
//     // Feedforward: scales with the desired cable length.
//     // Feedback: proportional to the cable length error.
//     const float cable_ff = 5.0;  // Feedforward gain (tune as needed)
//     const float cable_kp = 2.0;  // Proportional feedback gain (tune as needed)
    
//     float feedforward = cable_ff * desired_cable_length;
//     float error = desired_cable_length - current_cable_length;
//     float feedback = cable_kp * error;
    
//     float velocity_command = feedforward + feedback;
    
//     // Clamp velocity command to safe range, e.g., [-3, 3] degrees/sec.
//     if (velocity_command > 3.0)
//          velocity_command = 3.0;
//     else if (velocity_command < -3.0)
//          velocity_command = -3.0;
    
//     // If the cable length error is within a small tolerance, zero the velocity.
//     const float cable_tolerance = 0.005;  // meters; adjust as needed
//     if (fabs(error) < cable_tolerance) {
//          velocity_command = 0.0;
//     }
    
//     // --- Step 5: Flip the direction of the command ---
//     // (This inversion compensates for the current motor wiring/direction.)
//     //velocity_command = -velocity_command;
//     cout << "Velocity Command = " << velocity_command << endl;
//     // --- Step 6: Send the command to the pitch motor ---
//     // Here, we use the same interface as before.
//     pitch_motor.sendCommandMITMode(0, velocity_command, 0, 1.5, 0);
    
//     // --- Step 7: Check if both systems have reached their targets ---
//     const int zipper_tolerance = 100;  // encoder counts tolerance for the zipper motor
//     bool zipperReached = (abs(goal_count - _sz_encoder_count) <= zipper_tolerance);
//     bool cableReached = (fabs(error) < cable_tolerance);
    
//     return (zipperReached && cableReached);
// }

bool TurretController::actuateTurretCable(float goal_dist, float pitch, float yaw) {
    // --- Step 1: Update encoder counts and control the spiral zipper motor ---
    int goal_count = static_cast<int>(goal_dist / EXTENSION_PER_STEP / 4);
    updateSzEncoderCount();
    updateTurretEncoderCount();
    
    _sz_PWM = pController(goal_count, _sz_encoder_count);
    if (_sz_limit_switch_pressed && _sz_encoder_count > goal_count) {
         setMotorOutput(1500); // Stop zipper motor if limit reached.
    } else {
         setMotorOutput(_sz_PWM);
    }
    
    // --- Step 2: Compute current and desired cable lengths using geometry ---
    // The current cable length is computed using the current actuator extension (L)
    // and the measured turret pitch angle.
    float current_L = getActuatorLength();
    float measured_pitch = getTurretAngle();  // in degrees
    float current_r = x_offset + current_L * cos(measured_pitch * M_PI / 180.0);
    float current_theta = atan((y_offset - current_L * sin(measured_pitch * M_PI / 180.0)) /
                               (x_offset + current_L * cos(measured_pitch * M_PI / 180.0)));
    float current_cable_length = current_r * current_theta;
    
    // Compute the desired cable length from the goal zipper extension and desired pitch.
    float desired_r = x_offset + goal_dist * cos(pitch * M_PI / 180.0);
    float desired_theta = atan((y_offset - goal_dist * sin(pitch * M_PI / 180.0)) /
                               (x_offset + goal_dist * cos(pitch * M_PI / 180.0)));
    float desired_cable_length = desired_r * desired_theta;
    
    // --- Step 3: Compute the desired linear cable speed (in m/s) ---
    // Using a combination of a feedforward term (based on the desired cable length)
    // and a feedback term (proportional to the cable length error).
    const float cable_ff = 3.0;  // Feedforward gain [m/s per m] (tune as needed)
    const float cable_kp = 1.2;  // Proportional gain [m/s per m error] (tune as needed)
    float error = desired_cable_length - current_cable_length;
    float v_linear = cable_ff * desired_cable_length + cable_kp * error;  // Desired cable speed [m/s]
    
    // --- Step 4: Convert the linear speed to an angular velocity command ---
    // Given v_linear = ω * r, so ω = v_linear / r. With r = 0.013 m.
    float spool_radius = 0.013; // m (for a 26mm diameter spool)
    float omega_rad = v_linear / spool_radius;  // Angular speed in rad/s
    
    // If your motor expects degrees per second, convert from rad/s:
    float omega_deg = omega_rad * (180.0 / M_PI);
    
    // --- Step 5: Clamp the angular velocity command ---
    // For example, limit to a range of -3 to 3 deg/s (adjust as needed).
    if (omega_deg > 6.0)
         omega_deg = 6.0;
    else if (omega_deg < -6.0)
         omega_deg = -6.0;
    
    // If the cable length error is very small, set velocity to zero.
    const float cable_tolerance = 0.005;  // meters; adjust as needed
    if (fabs(error) < cable_tolerance) {
         omega_deg = 0.0;
    }
    
    // --- Step 6: Flip the direction if necessary ---
    // (Multiply by -1 if the motor rotation is reversed.)
    //omega_deg = -omega_deg;
    
    cout << "Velocity Command = " << omega_deg << endl;
    // --- Step 7: Command the pitch (cable) motor ---
    // Here we assume the motor control interface takes position, velocity, kp, kd, and torque.
    pitch_motor.sendCommandMITMode(0, omega_deg, 0, 1.5, 0);
    
    // --- Step 8: Check for target achievement ---
    const int zipper_tolerance = 100;  // encoder counts tolerance for zipper motor
    bool zipperReached = (abs(goal_count - _sz_encoder_count) <= zipper_tolerance);
    bool cableReached = (fabs(error) < cable_tolerance);
    
    return (zipperReached && cableReached);
}



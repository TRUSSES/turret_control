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

    cout << "DONE\n";
}


void TurretController::zeroZipper(){
	while (_sz_limit_switch_pressed == false) {
		gpioServo(_sz_esc_gpio_pin, 1650);
	}
	gpioServo(_sz_esc_gpio_pin, _sz_STOP_PWM);
	
	_sz_encoder_count = 0;
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

void TurretController::updateEncoderCount() {
    int current_encoder_value = readSZEncoder();

	 // Calculate the difference between the current and previous encoder values
    int difference = current_encoder_value - _prev_encoder_value;

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
    _prev_encoder_value = current_encoder_value;

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

    updateEncoderCount();
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
// void TurretController::actuateZipperLength(float goal_dist) {

//     // Convert the goal distance to the corresponding encoder count
//     int goal_count = static_cast<int>(goal_dist / EXTENSION_PER_STEP/4);

//     int _sz_PWM = (goal_count - _sz_encoder_count < 0) ? 1350 : 1650;

//     while (_sz_encoder_count != goal_count) {
//         std::cout << "Current count: " << _sz_encoder_count << " | Goal count: " << goal_count << "\n";

//         updateEncoderCount();

//         float desired_position = goal_count;
//         _sz_PWM = pController(desired_position, _sz_encoder_count);

//         setMotorOutput(_sz_PWM);
//         if (_sz_limit_switch_pressed && _sz_encoder_count > goal_count) {
//             break;
//         }
        
//         // rclcpp::spin_some(std::make_shared<rclcpp::Node>("turret0"));
//         sleep(0.1);
//     }

//     setMotorOutput(1500);  // Stop motor

// }

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


int TurretController::float_to_uint(float x, float x_min, float x_max, int bits)
{
    // Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12)
    {
        pgg = (unsigned int)((x - offset) * 4095.0 / span);
    }
    else if (bits == 16)
    {
        pgg = (unsigned int)((x - offset) * 65535.0 / span);
    }
    return pgg;
}

void TurretController::sendToTurretMotor(int motor_ID, float pos, float vel, float kp, float kd, float torq)
{
    printf("sending to motor \n");
    struct can_frame fr;
    memset(&fr, 0, sizeof(struct can_frame));

    float p_des = fminf(fmaxf(P_MIN, pos), P_MAX);
    float v_des = fminf(fmaxf(V_MIN, vel), V_MAX);
    kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
    kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
    float t_ff = fminf(fmaxf(T_MIN, torq), T_MAX);

    unsigned int con_pos = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int con_vel = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int con_kp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int con_kd = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int con_torq = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    fr.can_id = motor_ID;
    fr.can_dlc = 8;
    fr.data[0] = con_pos >> 8;
    fr.data[1] = con_pos & 0xFF;
    fr.data[2] = con_vel >> 4;
    fr.data[3] = ((con_vel & 0xF) << 4) | (con_kp >> 8);
    fr.data[4] = con_kp & 0xFF;
    fr.data[5] = con_kd >> 4;
    fr.data[6] = ((con_kd & 0xF) << 4) | (con_torq >> 8);
    fr.data[7] = con_torq & 0xFF;

    nbytes = write(sock, &fr, sizeof(fr));
    if (nbytes != sizeof(fr))
    {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }
}

void TurretController::activateTurretMotors(int motor_ID)
{
    printf("entering motor mode\n");
    struct can_frame cf;
    memset(&cf, 0, sizeof(struct can_frame));

    cf.can_id = motor_ID;
    cf.can_dlc = 8;
    cf.data[0] = 0xFF;
    cf.data[1] = 0xFF;
    cf.data[2] = 0xFF;
    cf.data[3] = 0xFF;
    cf.data[4] = 0xFF;
    cf.data[5] = 0xFF;
    cf.data[6] = 0xFF;
    cf.data[7] = 0xFC;

    nbytes = write(sock, &cf, sizeof(cf));
    if (nbytes != sizeof(cf))
    {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }
}

void TurretController::deactivateTurretMotors(int motor_ID)
{
    struct can_frame cf;
    memset(&cf, 0, sizeof(struct can_frame));

    cf.can_id = motor_ID;
    cf.can_dlc = 8;
    cf.data[0] = 0xFF;
    cf.data[1] = 0xFF;
    cf.data[2] = 0xFF;
    cf.data[3] = 0xFF;
    cf.data[4] = 0xFF;
    cf.data[5] = 0xFF;
    cf.data[6] = 0xFF;
    cf.data[7] = 0xFD;

    nbytes = write(sock, &cf, sizeof(cf));
    if (nbytes != sizeof(cf))
    {
        printf("Send Error frame[0]!\r\n");
        system("sudo ifconfig can0 down");
    }
}

TurretController::Cubemars_Motor TurretController::readTurretMotorValues(int motor_id)
{
    Cubemars_Motor motor;
    const int FEEDBACK_CAN_ID = motor_id;
    struct can_frame feedback_frame;

    const uint8_t MODE_COMMAND[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    struct can_frame mode_frame;
    mode_frame.can_id = motor_id;
    mode_frame.can_dlc = 8;
    std::memcpy(mode_frame.data, MODE_COMMAND, 8);

    if (write(sock, &mode_frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Error sending Servo Mode command");
        // close(socket_can);
        // return 1;
    }

    int ret_val = read(sock, &feedback_frame, sizeof(struct can_frame));
    if (ret_val < 0)
    {
        perror("CAN read error");
        //break;
    }

    // Check if this is the feedback message we are expecting
    if (feedback_frame.can_id == FEEDBACK_CAN_ID)
    {
        // Parse feedback data (adjust parsing based on your motor’s protocol)
        int motor_position = (feedback_frame.data[1] << 8) | feedback_frame.data[2];
        int motor_speed = (feedback_frame.data[3] << 4) | (feedback_frame.data[4] >> 4);
        int motor_current = ((feedback_frame.data[4] & 0x0F) << 8) | feedback_frame.data[5];
        int motor_temperature = feedback_frame.data[6];
        int motor_error_flag = feedback_frame.data[7];

        // Convert data to readable format (example scaling, adjust as needed)
        float position = static_cast<float>(motor_position) * 0.001; // Example scaling
        float speed = static_cast<float>(motor_speed) * 0.1;         // Example scaling
        float current = static_cast<float>(motor_current) * 0.01;    // Example scaling

        int motor_torque_raw = (feedback_frame.data[0] << 8) | feedback_frame.data[1];
        float motor_torque = static_cast<float>(motor_torque_raw) * 0.1;  // Example scaling to Nm

        // // Print feedback data
        // std::cout << "Motor ID = " << motor_id << ", "
        //           << "Feedback: Position = " << position << " rad, "
        //           << "Speed = " << speed << " rad/s, "
        //           << "Current = " << current << " A, "
        //           << "Temperature = " << motor_temperature - 40 << " °C, "
        //           << "Error Flag = " << motor_error_flag << std::endl;

        motor.motor_id = motor_id;
        motor.position = position;
        motor.speed = speed;
        motor.current = current;
        motor.temperature = motor_temperature - 40;
        motor.error_flag = motor_error_flag;
    }
    else
    {
        std::cout << "Received message with unexpected CAN ID: " << std::hex << feedback_frame.can_id << std::dec << std::endl;
    }

    return motor;
}

float TurretController::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void TurretController::closeSocket()
{
    close(sock);
}

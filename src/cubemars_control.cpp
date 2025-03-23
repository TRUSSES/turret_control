#include "cubemars_control.h"

CubemarsControl::CubemarsControl(){
    //do nothing 
}

//TODO: Split the socket construction from the motor control class and figure out what I need to pass it
CubemarsControl::CubemarsControl(int motor_id, int socket){
    // Default constructor
    motor_id_ = motor_id;
    motor_data_.motor_id = motor_id;
    motor_data_.position = -1;
    motor_data_.speed = -1;
    motor_data_.torque = -1;
    motor_data_.temperature = -1;
    motor_data_.error_flag = -1;
    sock_ = socket;
    clock_ = std::chrono::high_resolution_clock::now();
}

int CubemarsControl::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if(x<x_min){
        x=x_min;
    }
    else if (x > x_max){
        x = x_max;
    } 
    return (int) ((x-x_min)*((float)((1<<bits)/span)));
}

void CubemarsControl::toc(std::string word){
    // End timer (toc)
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate elapsed time
    std::chrono::duration<double> duration = end - clock_;
    std::cout << word << " " << duration.count()<< std::endl;

}

void CubemarsControl::sendCommandMITMode(float pos, float vel, float kp, float kd, float torq)
{
    //std::cout << "pos: " << pos << " vel: " << vel << " kp: " << kp << " kd: " << kd << " torq: " << torq << std::endl;
    //printf("sending to motor \n");
    struct can_frame fr;
    memset(&fr, 0, sizeof(struct can_frame));

    float p_des = fminf(fmaxf(P_MIN, pos), P_MAX);
    float v_des = fminf(fmaxf(V_MIN, vel), V_MAX);
    float kp_des = fminf(fmaxf(KP_MIN, kp), KP_MAX);
    float kd_des = fminf(fmaxf(KD_MIN, kd), KD_MAX);
    float t_ff = fminf(fmaxf(T_MIN, torq), T_MAX);

    int con_pos = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int con_vel = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int con_kp = float_to_uint(kp_des, KP_MIN, KP_MAX, 12);
    int con_kd = float_to_uint(kd_des, KD_MIN, KD_MAX, 12);
    int con_torq = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    //std::cout << "conpos: " << con_pos << " convel: " << con_vel << " conkp: " << con_kp << " conkd: " << con_kd << " contorq: " << con_torq << std::endl;
    fr.can_id = motor_id_;
    fr.can_dlc = 8;
    fr.data[0] = con_pos >> 8;
    fr.data[1] = con_pos & 0xFF;
    fr.data[2] = con_vel >> 4;
    fr.data[3] = ((con_vel & 0xF) << 4) | (con_kp >> 8);
    fr.data[4] = con_kp & 0xFF;
    fr.data[5] = con_kd >> 4;
    fr.data[6] = ((con_kd & 0xF) << 4) | (con_torq >> 8);
    fr.data[7] = con_torq & 0xFF;

    //clock_ = std::chrono::high_resolution_clock::now();
    nbytes_ = write(sock_, &fr, sizeof(fr));
    //toc("Written to CAN");

    if (nbytes_ != sizeof(fr))
    {
        printf("Send Error frame[0]!\r\n");
        //system("sudo ifconfig can0 down");
    }

    if(!waitForReply(kDefaultTimeoutUsecs)){
        std::cout << "No reply from motor" << std::endl;
    }
    //toc("Received and processed reply");
    //std::cout << "Motor ID: " << motor_data_.motor_id << " Position: " << motor_data_.position << " Speed: " << motor_data_.speed << " Temperature: " << motor_data_.temperature << " Error Flag: " << motor_data_.error_flag << std::endl;
}

bool CubemarsControl::waitForReply(int timeout_us) {
    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(sock_, &readfds);

    // Set timeout
    timeout.tv_sec = 0;
    timeout.tv_usec = timeout_us;
    //toc("Waiting for reply");
    int retval = select(sock_ + 1, &readfds, NULL, NULL, &timeout);
    //toc("Received reply");
    if (retval == -1) {
        perror("select() error");
        return false;
    } else if (retval == 0) {
        // Timeout reached
        printf("Timeout waiting for reply\n");
        return false;
    }

    // If data is available, call unpackReply
    return unpackReply();
}

bool CubemarsControl::unpackReply() {
    struct can_frame msg;
    int ret_val = read(sock_, &msg, sizeof(struct can_frame));
    time_of_last_read_ = std::chrono::high_resolution_clock::now();
    if (ret_val < 0) {
        perror("CAN read error");
        return false;
    } else if (ret_val == 0) {
        printf("No data available\n");
        return false;
    }
    if (msg.data[0] == motor_id_) {
        // Parse message data
        int p_int = (msg.data[1] << 8) | msg.data[2];
        int v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
        int i_int = ((msg.data[4] & 0x0F) << 8) | msg.data[5];
        int T_int = msg.data[6];

        motor_data_.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
        motor_data_.speed = uint_to_float(v_int, V_MIN, V_MAX, 12);
        motor_data_.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12);
        motor_data_.temperature = T_int - 40;
        motor_data_.error_flag = msg.data[7];
        
        return true;
    }else{
        std::cout << "Motor ID does not match" << std::endl;
        std::cout << "Motor ID: " << ((int) msg.data[0]) << " Expected ID: " << motor_id_ << std::endl;
    }
    
    return false;
}

void CubemarsControl::enterMITMode()
{
    struct can_frame cf;
    memset(&cf, 0, sizeof(struct can_frame));

    cf.can_id = motor_id_;
    cf.can_dlc = 8;
    cf.data[0] = 0xFF;
    cf.data[1] = 0xFF;
    cf.data[2] = 0xFF;
    cf.data[3] = 0xFF;
    cf.data[4] = 0xFF;
    cf.data[5] = 0xFF;
    cf.data[6] = 0xFF;
    cf.data[7] = 0xFC;

    nbytes_ = write(sock_, &cf, sizeof(cf));
    if (nbytes_ != sizeof(cf))
    {
        printf("Send Error frame[0]!\r\n");
        //system("sudo ifconfig can0 down");
    }
    if(!waitForReply(kDefaultTimeoutUsecs)){
        std::cout << "No reply from motor" << std::endl;
    }
}

void CubemarsControl::zeroMotor()
{
    struct can_frame cf;
    memset(&cf, 0, sizeof(struct can_frame));

    cf.can_id = motor_id_;
    cf.can_dlc = 8;
    cf.data[0] = 0xFF;
    cf.data[1] = 0xFF;
    cf.data[2] = 0xFF;
    cf.data[3] = 0xFF;
    cf.data[4] = 0xFF;
    cf.data[5] = 0xFF;
    cf.data[6] = 0xFF;
    cf.data[7] = 0xFE;

    nbytes_ = write(sock_, &cf, sizeof(cf));
    if (nbytes_ != sizeof(cf))
    {
        printf("Send Error frame[0]!\r\n");
        //system("sudo ifconfig can0 down");
    }
    if(!waitForReply(kZeroMotorTimeoutUsecs)){
        std::cout << "No reply from motor" << std::endl;
    }
}

void CubemarsControl::exitMITMode()
{
    struct can_frame cf;
    memset(&cf, 0, sizeof(struct can_frame));

    cf.can_id = motor_id_;
    cf.can_dlc = 8;
    cf.data[0] = 0xFF;
    cf.data[1] = 0xFF;
    cf.data[2] = 0xFF;
    cf.data[3] = 0xFF;
    cf.data[4] = 0xFF;
    cf.data[5] = 0xFF;
    cf.data[6] = 0xFF;
    cf.data[7] = 0xFD;

    nbytes_ = write(sock_, &cf, sizeof(cf));

    if (nbytes_ != sizeof(cf))
    {
        printf("Send Error frame[0]!\r\n");
        //system("sudo ifconfig can0 down");
    }

}

std::chrono::time_point<std::chrono::high_resolution_clock> CubemarsControl::getTimeOfLastRead()
{
    return time_of_last_read_;
}

float CubemarsControl::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


int CubemarsControl::getMotorID()
{
    return motor_id_;
}

CubemarsControl::Cubemars_Motor& CubemarsControl::getMotorData()
{
    return motor_data_;
}


#include "cubemars_pi3hat.h"
#include "cubemars_reply_router.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace {
// Commands for pitch and yaw share Pi3Hat's receive queue. This mutex protects
// both the transport and every registered motor's decoded feedback/cache.
std::mutex transport_mutex;
turret_control::CubemarsReplyRouter<CubemarsPi3Hat> reply_router;
}  // namespace

CubemarsPi3Hat::CubemarsPi3Hat(int motor_id, int can_bus, mjbots::pi3hat::Pi3Hat* pi3hat)
    : motor_id_(motor_id), can_bus_(5), pi3hat_(pi3hat)
{
    // The legacy can_bus argument selected a vector slot (currently 0), while
    // commands have always used physical JC5. Keep that constructor contract.
    (void)can_bus;
    tx_can_.resize(1);
    rx_can_.resize(32);
    std::lock_guard<std::mutex> lock(transport_mutex);
    reply_router.add(pi3hat_, can_bus_, motor_id_, this);
}

CubemarsPi3Hat::~CubemarsPi3Hat() {
    std::lock_guard<std::mutex> lock(transport_mutex);
    reply_router.remove(pi3hat_, can_bus_, motor_id_, this);
}

int CubemarsPi3Hat::float_to_uint(float x, float x_min, float x_max, int bits) {
    const int maximum = (1 << bits) - 1;
    const double span = static_cast<double>(x_max) - x_min;
    x = std::clamp(x, x_min, x_max);
    // Match the feedback decoder and keep endpoints inside their packed fields.
    const double scaled = (static_cast<double>(x) - x_min) * maximum / span;
    return static_cast<int>(std::clamp(std::round(scaled), 0.0,
                                       static_cast<double>(maximum)));
}

float CubemarsPi3Hat::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return static_cast<float>(x_int) * span / ((1 << bits) - 1) + x_min;
}

void CubemarsPi3Hat::sendCommandMITMode(float pos, float vel, float kp, float kd, float torq) {
    std::lock_guard<std::mutex> lock(transport_mutex);
    float p_des  = std::clamp(pos,  P_MIN, P_MAX);
    float v_des  = std::clamp(vel,  V_MIN, V_MAX);
    float kp_des = std::clamp(kp,   KP_MIN, KP_MAX);
    float kd_des = std::clamp(kd,   KD_MIN, KD_MAX);
    float t_ff   = std::clamp(torq, T_MIN, T_MAX);

    int con_pos  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int con_vel  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int con_kp   = float_to_uint(kp_des, KP_MIN, KP_MAX, 12);
    int con_kd   = float_to_uint(kd_des, KD_MIN, KD_MAX, 12);
    int con_torq = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    mjbots::pi3hat::CanFrame &frame = tx_can_[0];
    frame.id = motor_id_;
    frame.bus = can_bus_;  // Physical JC5.
    frame.size = 8;
    frame.expect_reply = true;
    frame.data[0] = static_cast<uint8_t>(con_pos >> 8);
    frame.data[1] = static_cast<uint8_t>(con_pos & 0xFF);
    frame.data[2] = static_cast<uint8_t>(con_vel >> 4);
    frame.data[3] = static_cast<uint8_t>(((con_vel & 0xF) << 4) | (con_kp >> 8));
    frame.data[4] = static_cast<uint8_t>(con_kp & 0xFF);
    frame.data[5] = static_cast<uint8_t>(con_kd >> 4);
    frame.data[6] = static_cast<uint8_t>(((con_kd & 0xF) << 4) | (con_torq >> 8));
    frame.data[7] = static_cast<uint8_t>(con_torq & 0xFF);

    cycleAndDispatchReplies();
}

void CubemarsPi3Hat::cycleAndDispatchReplies() {
    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << can_bus_);
    input.timeout_ns = 1200000;

    auto output = pi3hat_->Cycle(input);
    for (size_t i = 0; i < output.rx_can_size; i++) {
        reply_router.dispatch(pi3hat_, rx_can_[i],
            [](CubemarsPi3Hat& motor, const mjbots::pi3hat::CanFrame& reply) {
                motor.observeBusFrame(reply);
            },
            [](CubemarsPi3Hat& motor, const mjbots::pi3hat::CanFrame& reply) {
                motor.unpackReply(reply);
            });
    }
}

void CubemarsPi3Hat::enterMITMode() {
    std::lock_guard<std::mutex> lock(transport_mutex);
    sendModeCommand(0xFC);
    std::cout << "CubemarsPi3Hat::enterMITMode() - command sent for Motor ID: " << motor_id_
              << "; mode acknowledgement not implied" << std::endl;
}

void CubemarsPi3Hat::exitMITMode() {
    std::lock_guard<std::mutex> lock(transport_mutex);
    sendModeCommand(0xFD);
}

void CubemarsPi3Hat::zeroMotor() {
    std::lock_guard<std::mutex> lock(transport_mutex);
    sendModeCommand(0xFE);
}

void CubemarsPi3Hat::sendModeCommand(std::uint8_t command) {
    mjbots::pi3hat::CanFrame &frame = tx_can_[0];
    frame.id = motor_id_;
    frame.bus = can_bus_;
    frame.size = 8;
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = command;
    frame.expect_reply = false;

    cycleAndDispatchReplies();
}

void CubemarsPi3Hat::observeBusFrame(const mjbots::pi3hat::CanFrame &frame) {
    ++bus_frames_seen_;
    last_frame_size_ = frame.size;
    last_payload_motor_id_ = frame.size > 0 ? frame.data[0] : -1;
    last_frame_can_id_ = frame.id;
}

bool CubemarsPi3Hat::unpackReply(const mjbots::pi3hat::CanFrame &frame) {
    if (frame.bus != can_bus_ || frame.size == 0 ||
        frame.data[0] != static_cast<uint8_t>(motor_id_)) {
        return false;
    }
    ++addressed_frames_seen_;
    last_addressed_frame_size_ = frame.size;
    if (frame.size != 8) {
        ++rejected_size_frames_;
        return false;
    }

    int p_int = (frame.data[1] << 8) | frame.data[2];
    int v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    int i_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];

    motor_data_.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    motor_data_.velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_data_.torque   = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    motor_data_.temperature = static_cast<float>(frame.data[6]) - 40;
    motor_data_.error_flag = frame.data[7];
    last_feedback_time_ = std::chrono::steady_clock::now();
    have_feedback_ = true;
    ++feedback_sequence_;

    return true;
}

float CubemarsPi3Hat::getPosition() const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    return motor_data_.position;
}
float CubemarsPi3Hat::getVelocity() const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    return motor_data_.velocity;
}
float CubemarsPi3Hat::getTorque() const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    return motor_data_.torque;
}
float CubemarsPi3Hat::getTemperature() const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    return motor_data_.temperature;
}
int CubemarsPi3Hat::getErrorFlag() const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    return motor_data_.error_flag;
}
std::uint64_t CubemarsPi3Hat::GetFeedbackSequence() const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    return feedback_sequence_;
}

bool CubemarsPi3Hat::HasRecentFeedback(double max_age_sec) const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    if (!have_feedback_ || !std::isfinite(max_age_sec) || max_age_sec <= 0.0) {
        return false;
    }
    const double age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_feedback_time_).count();
    return age_sec >= 0.0 && age_sec <= max_age_sec;
}

std::string CubemarsPi3Hat::GetFeedbackDiagnostics() const {
    std::lock_guard<std::mutex> lock(transport_mutex);
    std::ostringstream text;
    text << "motor_id=" << motor_id_ << " bus=" << can_bus_
         << " accepted_replies=" << feedback_sequence_ << " last_reply_age_ms=";
    if (have_feedback_) {
        const auto age = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - last_feedback_time_).count();
        text << std::fixed << std::setprecision(1) << age;
    } else {
        text << "none";
    }
    text << " bus_frames_seen=" << bus_frames_seen_
         << " motor_id_frames_seen=" << addressed_frames_seen_
         << " rejected_size_frames=" << rejected_size_frames_
         << " last_motor_frame_size=" << last_addressed_frame_size_
         << " last_bus_frame_size=" << last_frame_size_
         << " last_payload_motor_id=" << last_payload_motor_id_
         << " last_can_id=" << last_frame_can_id_;
    return text.str();
}

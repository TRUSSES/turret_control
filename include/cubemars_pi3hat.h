#ifndef CUBEMARS_PI3HAT_H
#define CUBEMARS_PI3HAT_H

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include "pi3hat.h"

// CubeMars motor control via Pi3Hat CAN interface (MIT mode)
// This is separate from the SocketCAN-based CubemarsControl class
class CubemarsPi3Hat {
public:
    CubemarsPi3Hat(int motor_id, int can_bus, mjbots::pi3hat::Pi3Hat* pi3hat);
    ~CubemarsPi3Hat();
    CubemarsPi3Hat(const CubemarsPi3Hat&) = delete;
    CubemarsPi3Hat& operator=(const CubemarsPi3Hat&) = delete;

    // MIT Mode command functions
    void sendCommandMITMode(float pos, float vel, float kp, float kd, float torq);
    void enterMITMode();
    void exitMITMode();
    void zeroMotor();

    // Getters for feedback
    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;
    float getTemperature() const;
    int getErrorFlag() const;
    bool HasRecentFeedback(double max_age_sec) const;
    // Advances once per valid decoded reply, not once per command or getter.
    std::uint64_t GetFeedbackSequence() const;
    std::string GetFeedbackDiagnostics() const;

private:
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    bool unpackReply(const mjbots::pi3hat::CanFrame &frame);
    // Called only while the shared transport mutex is held.
    void cycleAndDispatchReplies();
    void sendModeCommand(std::uint8_t command);
    void observeBusFrame(const mjbots::pi3hat::CanFrame &frame);

    // Parameter limits for MIT mode
    static constexpr float P_MIN = -95.5f;
    static constexpr float P_MAX = +95.5f;
    static constexpr float V_MIN = -30.0f;
    static constexpr float V_MAX = +30.0f;
    static constexpr float T_MIN = -18.0f;
    static constexpr float T_MAX = +18.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;

    int motor_id_;
    int can_bus_;
    mjbots::pi3hat::Pi3Hat* pi3hat_;

    struct MotorData {
        float position = 0.0f;
        float velocity = 0.0f;
        float torque = 0.0f;
        float temperature = 0.0f;
        int error_flag = 0;
    } motor_data_;

    bool have_feedback_ = false;
    std::uint64_t feedback_sequence_ = 0;
    std::chrono::steady_clock::time_point last_feedback_time_;
    std::uint64_t bus_frames_seen_ = 0;
    std::uint64_t addressed_frames_seen_ = 0;
    std::uint64_t rejected_size_frames_ = 0;
    int last_frame_size_ = -1;
    int last_payload_motor_id_ = -1;
    std::uint32_t last_frame_can_id_ = 0;
    int last_addressed_frame_size_ = -1;

    std::vector<mjbots::pi3hat::CanFrame> tx_can_;
    std::vector<mjbots::pi3hat::CanFrame> rx_can_;
};

#endif // CUBEMARS_PI3HAT_H

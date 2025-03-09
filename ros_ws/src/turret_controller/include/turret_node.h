#ifndef TURRET_NODE_H
#define TURRET_NODE_H

#include "TurretController.h"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include "trusses_custom_interfaces/srv/set_value.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trusses_custom_interfaces/srv/actuator_enable.hpp"
#include "trusses_custom_interfaces/srv/set_string.hpp"
#include "trusses_custom_interfaces/srv/set_value_array.hpp"

#include "chrono"
#include "string"
#include <cstdlib>
#include <map>

using namespace std::chrono_literals;

class TurretNode : public rclcpp::Node {
public:
    explicit TurretNode(int turret_number);
    void cleanup();

    TurretController turret;

private:

    float desired_zipper_length_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr load_cell_timer_;


    float pitch_offset_ = 0.0;
    float yaw_offset_ = 0.0;

    float zipper_P_ = 5.0;
    float pitch_P_ = 1.0;
    float pitch_D_ = 2.0;
    float yaw_P_ = 1.0;
    float yaw_D_ = 2.0;

    std::string behavior_ = "idle";

    enum BehaviorTypes{
        kIdle,
        kPosition
    };

    std::map<std::string, BehaviorTypes> behavior_map_ = {
        {"idle", kIdle},
        {"position", kPosition}
    };

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_zipper_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_all_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetValue>::SharedPtr set_pitch_offset_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetValue>::SharedPtr set_yaw_offset_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetValue>::SharedPtr set_zipper_P_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetString>::SharedPtr set_behavior_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetValueArray>::SharedPtr set_pitch_PD_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetValueArray>::SharedPtr set_yaw_PD_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::ActuatorEnable>::SharedPtr set_actuator_enabled_service_;

    void control_loop();

    // Service callbacks
    void zero_zipper_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void stop_all_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void set_pitch_offset_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Response> response);

    void set_yaw_offset_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Response> response);

    void set_zipper_P_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Response> response);

    void set_pitch_PD_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Response> response);

    void set_yaw_PD_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Response> response);

    void set_actuator_enabled_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::ActuatorEnable::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::ActuatorEnable::Response> response);

    void set_behavior_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetString::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetString::Response> response);


    void publish_curr_state();
    void publish_load_cell_state();

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr curr_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr load_cell_state_pub_;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr set_states_sub_;
    void set_states_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};

#endif
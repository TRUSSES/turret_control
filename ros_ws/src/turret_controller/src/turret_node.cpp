#include "turret_node.h"

TurretNode::TurretNode(int turret_number) : Node("turret" + std::to_string(turret_number))
{

    // Initialize Services
    zero_zipper_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/zero_zipper", std::bind(&TurretNode::zero_zipper_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_all_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_all", std::bind(&TurretNode::stop_all_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_pitch_offset_service_ = this->create_service<trusses_custom_interfaces::srv::SetValue>(
        "~/set_pitch_offset", std::bind(&TurretNode::set_pitch_offset_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_yaw_offset_service_ = this->create_service<trusses_custom_interfaces::srv::SetValue>(
        "~/set_yaw_offset", std::bind(&TurretNode::set_yaw_offset_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_zipper_P_service_ = this->create_service<trusses_custom_interfaces::srv::SetValue>(
        "~/set_zipper_P", std::bind(&TurretNode::set_zipper_P_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_pitch_PD_service_ = this->create_service<trusses_custom_interfaces::srv::SetValueArray>(
        "~/set_pitch_PD", std::bind(&TurretNode::set_pitch_PD_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_yaw_PD_service_ = this->create_service<trusses_custom_interfaces::srv::SetValueArray>(
        "~/set_yaw_PD", std::bind(&TurretNode::set_yaw_PD_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_actuator_enabled_service_ = this->create_service<trusses_custom_interfaces::srv::ActuatorEnable>(
        "~/set_actuator_enabled", std::bind(&TurretNode::set_actuator_enabled_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_behavior_service_ = this->create_service<trusses_custom_interfaces::srv::SetString>(
        "~/set_behavior", std::bind(&TurretNode::set_behavior_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize Publishers
    curr_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/curr_state", 10);
    load_cell_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/load_cell_state", 10);

    // Initialize Subscriber
    set_states_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "~/set_states", 10,
        std::bind(&TurretNode::set_states_callback, this, std::placeholders::_1));

    // Initialize Turret
    turret.initTurret();
    sleep(1);

    // Timer for Publishers at 100 Hz
    state_timer_ = this->create_wall_timer(
        100ms, // 10 Hz frequency
        std::bind(&TurretNode::publish_curr_state, this));

    // Timer for Publishers at 100 Hz
    load_cell_timer_ = this->create_wall_timer(
        100ms, // 10 Hz frequency
        std::bind(&TurretNode::publish_load_cell_state, this));

    // Timer for Publishers at 100 Hz
    control_timer_ = this->create_wall_timer(
        10ms, // 10 Hz frequency
        std::bind(&TurretNode::control_loop, this));
}

void TurretNode::control_loop()
{

    switch (behavior_map_[behavior_])
    {
    default:
    case kIdle:
        turret.updateEncoderCount();
        turret.stopMotor();
        break;
    case kPosition:
        turret.updateEncoderCount();
        turret.actuateZipperLength(desired_zipper_length_);
        break;
    }
}

void TurretNode::set_behavior_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetString::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetString::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Setting behavior to: %s", request->data.c_str());

    behavior_ = request->data;
}

void TurretNode::zero_zipper_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received zero_zipper request.");
    // turret.initTurret();
    turret.zeroZipper();

    response->success = true;
}

void TurretNode::stop_all_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received stop_all request.");

    // turret.stopMotor();
    response->success = true;
}

void TurretNode::set_pitch_offset_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Response> response)
{
    // RCLCPP_INFO(this->get_logger(), "Setting pitch offset to: %f", request->data);
    pitch_offset_ = request->data - turret.readTurretEncoder();
}

void TurretNode::set_yaw_offset_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Response> response)
{
    // RCLCPP_INFO(this->get_logger(), "Setting yaw offset to: %f", request->data);
    // RCLCPP_WARN(this->get_logger(), "Yaw reading not implemented yet.");
    //  TODO : read yaw motor data
    // yaw_offset_ = request->data - turret.readEncoder();
}

void TurretNode::set_zipper_P_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetValue::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Setting zipper P value to: %f", request->data);
    zipper_P_ = request->data;
}

void TurretNode::set_pitch_PD_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Setting pitch PD values.");
    pitch_P_ = request->data[0];
    pitch_D_ = request->data[1];
}

void TurretNode::set_yaw_PD_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Response> response)
{
    // RCLCPP_INFO(this->get_logger(), "Setting yaw PD values.");
    yaw_P_ = request->data[0];
    yaw_D_ = request->data[1];
}

void TurretNode::set_actuator_enabled_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::ActuatorEnable::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::ActuatorEnable::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Setting actuator enabled state to: %s", request->enable ? "true" : "false");

    if (request->actuator_name == "zipper")
    {
        if (request->enable)
        {
            // turret.initTurret();
        }
        else
        {
            // turret.stopMotor();
        }
    }
    else if (request->actuator_name == "pitch")
    {
        if (request->enable)
        {
            // turret.enterMotorMode(2);
        }
        else
        {
            // turret.exitMotorMode(2);
        }
    }
    else if (request->actuator_name == "yaw")
    {
        if (request->enable)
        {
            // turret.enterMotorMode(41);
        }
        else
        {
            // turret.exitMotorMode(41);
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid actuator name.");
    }
}

void TurretNode::set_states_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received state update.");
    if (msg->data.size() != 3)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid state update message.");
        return;
    }

    if (msg->data[0] >= 0)
    {
        RCLCPP_INFO(this->get_logger(), "Setting zipper length to: %f", msg->data[0]);
        desired_zipper_length_ = msg->data[0];
        // turret.actuateZipperLength(msg->data[0]);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid zipper length. Must be greater than zero. Received: %f", msg->data[0]);
    }

    // RCLCPP_WARN(this->get_logger(), "Pitch motor not implemented yet.");
    /*
    RCLCPP_INFO(this->get_logger(), "Setting pitch angle to: %f", msg->data[1]);
    double pitch_diff = msg->data[1] - turret.readEncoder() - pitch_offset_;
    if (pitch_diff > M_PI) {
        pitch_diff -= 2 * M_PI;
    } else if (pitch_diff < -M_PI) {
        pitch_diff += 2 * M_PI;
    }

    if(pitch_diff > 0) {
        turret.sendToMotor(2, msg->data[1]-pitch_offset_, 2, pitch_P_, pitch_D_, 0);
    } else {
        turret.sendToMotor(2, msg->data[1]-pitch_offset_, -2, pitch_P_, pitch_D_, 0);
    }
    */

    // RCLCPP_WARN(this->get_logger(), "Yaw motor not implemented yet.");
    // Need to get value straight from the motor
    /*
    RCLCPP_INFO(this->get_logger(), "Setting yaw angle to: %f", msg->data[2]);
    double yaw_diff = msg->data[2] - turret.readEncoder() - yaw_offset_;
    if (yaw_diff > M_PI) {
        yaw_diff -= 2 * M_PI;
    } else if (yaw_diff < -M_PI) {
        yaw_diff += 2 * M_PI;
    }

    if(yaw_diff > 0) {
        turret.sendToMotor(41, msg->data[2]-yaw_offset_, 2, yaw_P_, yaw_D_, 0);
    } else {
        turret.sendToMotor(41, msg->data[2]-yaw_offset_, -2, yaw_P_, yaw_D_, 0);
    }
    */
}

void TurretNode::publish_curr_state()
{
    auto message = std_msgs::msg::Float32MultiArray();
    // RCLCPP_WARN(this->get_logger(), "Yaw motor reading (and offset) not implemented yet.");
    float motor_pitch = -1.0;
    float motor_yaw = -1.0;
    message.data = {(float)turret.getActuatorLength(), motor_pitch, motor_yaw};
    curr_state_pub_->publish(message);
}

void TurretNode::publish_load_cell_state()
{
    auto message = std_msgs::msg::Float32MultiArray();
    // message.data = {(float)turret.loadCell1.load(std::memory_order_relaxed), (float)turret.loadCell2.load(std::memory_order_relaxed), (float)turret.loadCell3.load(std::memory_order_relaxed)};
    message.data.resize(3);
    message.data[0] = static_cast<float>(turret.getLoadCell1Val());
    message.data[1] = static_cast<float>(turret.getLoadCell2Val());
    message.data[2] = static_cast<float>(turret.getLoadCell3Val());
    load_cell_state_pub_->publish(message);
}

void TurretNode::cleanup()
{
    // turret.stopMotor();
    // turret.exitMotorMode(2);
    // turret.exitMotorMode(41);
    // turret.closeSocket();
}

std::atomic_bool g_terminate_requested(false);

// Signal handler function
void signal_handler(int signum)
{
    if (signum == SIGINT)
    {
        g_terminate_requested.store(true);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Termination signal (CTRL-c) received. Shutting down...");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    int turret_number = -1; // Default value

    // Loop through argv to find the custom argument
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg == "--turret_number" && i + 1 < argc)
        {
            turret_number = std::atoi(argv[i + 1]);
            i++;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turret number: %d", turret_number);

    // Create and spin the node
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);

    auto turret_node = std::make_shared<TurretNode>(turret_number);
    executor->add_node(turret_node);

    rclcpp::on_shutdown([turret_node]()
                        { turret_node->cleanup(); });

    executor->spin();
    rclcpp::shutdown();
    return 0;
}
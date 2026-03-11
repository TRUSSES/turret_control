#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "turret_control/msg/turret_state.hpp"
#include "turret_control/msg/zipper_command.hpp"
#include "turret_control/msg/turret_teleop_command.hpp"
#include "turret_control/msg/turret_velocities.hpp"
#include "turret_control/msg/load_cell_force.hpp"
#include "turret_control/srv/zero_turret.hpp"
#include "turret_control/srv/set_state.hpp"
#include "turret.h"
#include "config.h"
#include <chrono>
#include <memory>
#include <future>
#include <algorithm>
#include <cmath>
#include <signal.h>
#include <pigpio.h>

using namespace std::chrono_literals;

class TurretROS2Node : public rclcpp::Node
{
private:
    static std::string createNodeName() {
        // Try to read turret_id from config to create node name
        // Default config paths to try
        std::vector<std::string> config_paths = {
            "config/config.yaml",
            "../config/config.yaml", 
            "/home/turret/ros_ws/src/turret_control/config/config.yaml"
        };
        
        int turret_id = 1; // Default
        for (const auto& config_path : config_paths) {
            try {
                YAML::Node config = YAML::LoadFile(config_path);
                if (config["turret_id"]) {
                    turret_id = config["turret_id"].as<int>();
                    break;
                }
            } catch (...) {
                // Continue to next path
            }
        }
        
        return "turret_control_node_" + std::to_string(turret_id);
    }

public:
    TurretROS2Node() : Node(createNodeName()), turret_id_(1) // Default turret_id
    {
        // pigpio is initialised in main() before this constructor runs

        // Declare parameter for config path
        this->declare_parameter("config_path", "config/config.yaml");
        
        // Declare zero velocity parameter
        this->declare_parameter("zero_velocity", -0.2);
        this->declare_parameter("docking_standoff_m", 0.30);
        this->declare_parameter("docking_extension_max_m", 0.90);
        this->declare_parameter("docking_pitch_limit_deg", 60.0);
        this->declare_parameter("docking_pitch_hold_cap_deg", 50.0);
        this->declare_parameter("docking_command_velocity", 1.5);
        this->declare_parameter("docking_stage_extension_m", 0.05);
        this->declare_parameter("docking_stage_pitch_deg", 25.0);
        this->declare_parameter("docking_stage_pitch_tolerance_deg", 3.0);
        this->declare_parameter("docking_stage_min_pitch_deg", 18.0);
        this->declare_parameter("docking_stage_extension_tolerance_m", 0.005);
        this->declare_parameter("docking_camera_filter_alpha", 0.2);
        this->declare_parameter("docking_pose_timeout_sec", 0.5);
        this->declare_parameter("docking_lateral_tolerance_m", 0.05);
        this->declare_parameter("docking_hold_on_lateral_error", false);
        this->declare_parameter("docking_lock_goal_camera_y_on_stage_complete", true);
        this->declare_parameter("docking_goal_camera_y_offset_m", -0.10);
        this->declare_parameter("docking_stop_on_close_tracking_loss", true);
        this->declare_parameter("docking_tracking_loss_stop_distance_m", 0.25);
        this->declare_parameter("docking_camera_forward_sign", 1.0);
        this->declare_parameter("docking_camera_vertical_sign", 1.0);
        this->declare_parameter("docking_visual_extension_kp", 0.8);
        this->declare_parameter("docking_visual_pitch_kp", 10.0);
        this->declare_parameter("docking_visual_max_extension_rate_mps", 0.12);
        this->declare_parameter("docking_visual_max_pitch_rate_deg_s", 35.0);
        this->declare_parameter("docking_visual_distance_deadband_m", 0.01);
        this->declare_parameter("docking_visual_vertical_deadband_m", 0.01);
        this->declare_parameter("docking_visual_command_period_sec", 0.25);
        this->declare_parameter("docking_visual_pitch_command_horizon_sec", 0.25);
        this->declare_parameter("docking_visual_extension_step_max_m", 0.08);
        this->declare_parameter("docking_visual_pitch_step_max_deg", 8.0);
        this->declare_parameter("docking_visual_slowdown_start_m", 0.30);
        this->declare_parameter("docking_visual_near_goal_scale", 0.25);
        this->declare_parameter("docking_visual_vertical_priority_error_m", 0.02);
        this->declare_parameter("docking_visual_pitch_limit_guard_deg", 15.0);
        
        // Load configuration - try different paths
        std::vector<std::string> config_paths = {
            this->get_parameter("config_path").as_string(),
            "config/config.yaml",
            "../config/config.yaml", 
            "/home/turret/ros_ws/src/turret_control/config/config.yaml"
        };
        
        bool config_loaded = false;
        for (const auto& config_path : config_paths) {
            if (Config::Instance().Load(config_path)) {
                RCLCPP_INFO(this->get_logger(), "Loaded configuration from %s", config_path.c_str());
                config_loaded = true;
                break;
            }
        }
        
        if (!config_loaded) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load configuration from any attempted path");
            throw std::runtime_error("Configuration file not found");
        }
        config_ = Config::Instance().GetConfig();
        if (config_["docking"]) {
            const auto docking = config_["docking"];
            if (docking["standoff_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_standoff_m", docking["standoff_m"].as<double>()));
            }
            if (docking["extension_max_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_extension_max_m", docking["extension_max_m"].as<double>()));
            }
            if (docking["pitch_limit_deg"]) {
                this->set_parameter(rclcpp::Parameter("docking_pitch_limit_deg", docking["pitch_limit_deg"].as<double>()));
            }
            if (docking["command_velocity"]) {
                this->set_parameter(rclcpp::Parameter("docking_command_velocity", docking["command_velocity"].as<double>()));
            }
            if (docking["stage_extension_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_stage_extension_m", docking["stage_extension_m"].as<double>()));
            }
            if (docking["stage_pitch_deg"]) {
                this->set_parameter(rclcpp::Parameter("docking_stage_pitch_deg", docking["stage_pitch_deg"].as<double>()));
            }
            if (docking["stage_pitch_tolerance_deg"]) {
                this->set_parameter(rclcpp::Parameter("docking_stage_pitch_tolerance_deg", docking["stage_pitch_tolerance_deg"].as<double>()));
            }
            if (docking["stage_min_pitch_deg"]) {
                this->set_parameter(rclcpp::Parameter("docking_stage_min_pitch_deg", docking["stage_min_pitch_deg"].as<double>()));
            }
            if (docking["stage_extension_tolerance_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_stage_extension_tolerance_m", docking["stage_extension_tolerance_m"].as<double>()));
            }
            if (docking["camera_filter_alpha"]) {
                this->set_parameter(rclcpp::Parameter("docking_camera_filter_alpha", docking["camera_filter_alpha"].as<double>()));
            }
            if (docking["pose_timeout_sec"]) {
                this->set_parameter(rclcpp::Parameter("docking_pose_timeout_sec", docking["pose_timeout_sec"].as<double>()));
            }
            if (docking["lateral_tolerance_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_lateral_tolerance_m", docking["lateral_tolerance_m"].as<double>()));
            }
            if (docking["hold_on_lateral_error"]) {
                this->set_parameter(rclcpp::Parameter("docking_hold_on_lateral_error", docking["hold_on_lateral_error"].as<bool>()));
            }
            if (docking["lock_goal_camera_y_on_stage_complete"]) {
                this->set_parameter(rclcpp::Parameter("docking_lock_goal_camera_y_on_stage_complete", docking["lock_goal_camera_y_on_stage_complete"].as<bool>()));
            }
            if (docking["goal_camera_y_offset_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_goal_camera_y_offset_m", docking["goal_camera_y_offset_m"].as<double>()));
            }
            if (docking["stop_on_close_tracking_loss"]) {
                this->set_parameter(rclcpp::Parameter("docking_stop_on_close_tracking_loss", docking["stop_on_close_tracking_loss"].as<bool>()));
            }
            if (docking["tracking_loss_stop_distance_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_tracking_loss_stop_distance_m", docking["tracking_loss_stop_distance_m"].as<double>()));
            }
            if (docking["camera_forward_sign"]) {
                this->set_parameter(rclcpp::Parameter("docking_camera_forward_sign", docking["camera_forward_sign"].as<double>()));
            }
            if (docking["camera_vertical_sign"]) {
                this->set_parameter(rclcpp::Parameter("docking_camera_vertical_sign", docking["camera_vertical_sign"].as<double>()));
            }
            if (docking["visual_extension_kp"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_extension_kp", docking["visual_extension_kp"].as<double>()));
            }
            if (docking["visual_pitch_kp"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_pitch_kp", docking["visual_pitch_kp"].as<double>()));
            }
            if (docking["visual_max_extension_rate_mps"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_max_extension_rate_mps", docking["visual_max_extension_rate_mps"].as<double>()));
            }
            if (docking["visual_max_pitch_rate_deg_s"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_max_pitch_rate_deg_s", docking["visual_max_pitch_rate_deg_s"].as<double>()));
            }
            if (docking["visual_distance_deadband_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_distance_deadband_m", docking["visual_distance_deadband_m"].as<double>()));
            }
            if (docking["visual_vertical_deadband_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_vertical_deadband_m", docking["visual_vertical_deadband_m"].as<double>()));
            }
            if (docking["visual_command_period_sec"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_command_period_sec", docking["visual_command_period_sec"].as<double>()));
            }
            if (docking["visual_pitch_command_horizon_sec"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_pitch_command_horizon_sec", docking["visual_pitch_command_horizon_sec"].as<double>()));
            }
            if (docking["visual_extension_step_max_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_extension_step_max_m", docking["visual_extension_step_max_m"].as<double>()));
            }
            if (docking["visual_pitch_step_max_deg"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_pitch_step_max_deg", docking["visual_pitch_step_max_deg"].as<double>()));
            }
            if (docking["visual_slowdown_start_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_slowdown_start_m", docking["visual_slowdown_start_m"].as<double>()));
            }
            if (docking["visual_near_goal_scale"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_near_goal_scale", docking["visual_near_goal_scale"].as<double>()));
            }
            if (docking["visual_vertical_priority_error_m"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_vertical_priority_error_m", docking["visual_vertical_priority_error_m"].as<double>()));
            }
            if (docking["visual_pitch_limit_guard_deg"]) {
                this->set_parameter(rclcpp::Parameter("docking_visual_pitch_limit_guard_deg", docking["visual_pitch_limit_guard_deg"].as<double>()));
            }
        }

        // Read turret_id from config
        turret_id_ = config_["turret_id"] ? config_["turret_id"].as<int>() : 1;
        RCLCPP_INFO(this->get_logger(), "Turret ID: %d", turret_id_);
        
        // Create topic prefix based on turret_id
        topic_prefix_ = "turret" + std::to_string(turret_id_);
        
        // Log the topic prefix for user information
        RCLCPP_INFO(this->get_logger(), "Using topic prefix: %s", topic_prefix_.c_str());

        // Initialize turret with config
        try {
            turret_ = std::make_unique<Turret>(config_);
            turret_->Init();
            RCLCPP_INFO(this->get_logger(), "Turret initialized successfully");
            RCLCPP_INFO(this->get_logger(), "Initial turret limit switch state: %s",
                        turret_->IsTurretLimitPressed() ? "PRESSED" : "RELEASED");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize turret: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Initialize state machine FIRST - we'll be in IDLE during load cell init
        current_state_ = TurretState::IDLE;
        is_zeroed_ = false;
        is_zeroing_ = false;
        sz_zeroed_ = false;
        pitch_zeroed_ = false;
        yaw_zeroed_ = false;
        current_extension_ = 0.0;
        current_velocity_ = 0.0;
        desired_length_ = 0.0;  // Initialize to prevent oscillation
        desired_velocity_ = 0.0;  // Initialize to prevent oscillation
        desired_pitch_angle_ = 0.0;
        hold_current_pitch_ = true;
        // Keep a conservative internal default for spiral zipper zeroing.
        // Actual runtime value comes from the "zero_velocity" parameter.
        zero_velocity_ = -0.2;
        
        RCLCPP_INFO(this->get_logger(), "State machine initialized - State: IDLE, Zeroed: %s", 
            is_zeroed_ ? "true" : "false");
        
        // Publishers
        heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>(topic_prefix_ + "/heartbeat", 10);
        state_pub_ = this->create_publisher<turret_control::msg::TurretState>(topic_prefix_ + "/state", 10);
        velocities_pub_ = this->create_publisher<turret_control::msg::TurretVelocities>(
            "cmd_vel/" + topic_prefix_ + "/velocities", 10);
        docking_camera_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_prefix_ + "/docking/camera_estimate_base", 10);
        docking_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_prefix_ + "/docking/target_pose_base", 10);

        // Subscribers
        zipper_cmd_sub_ = this->create_subscription<turret_control::msg::ZipperCommand>(
            topic_prefix_ + "/zipper_command", 10,
            std::bind(&TurretROS2Node::zipperCommandCallback, this, std::placeholders::_1));

        teleop_cmd_sub_ = this->create_subscription<turret_control::msg::TurretTeleopCommand>(
            topic_prefix_ + "/teleop_command", 10,
            std::bind(&TurretROS2Node::teleopCommandCallback, this, std::placeholders::_1));

        // Subscribe to load cell force data published by LoadCellNode
        load_cell_force_sub_ = this->create_subscription<turret_control::msg::LoadCellForce>(
            topic_prefix_ + "/load_cell_force", 10,
            std::bind(&TurretROS2Node::loadCellForceCallback, this, std::placeholders::_1));
        vision_tag_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/end_effector_tag_pose_camera", 10,
            std::bind(&TurretROS2Node::visionTagPoseCallback, this, std::placeholders::_1));
        vision_tag_visible_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/vision/tag_visible", 10,
            std::bind(&TurretROS2Node::visionTagVisibleCallback, this, std::placeholders::_1));

        // Services
        try {
            zero_service_ = this->create_service<turret_control::srv::ZeroTurret>(
                topic_prefix_ + "/zero",
                std::bind(&TurretROS2Node::zeroTurretCallback, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Zero service created successfully: /%s/zero", topic_prefix_.c_str());
            
            set_state_service_ = this->create_service<turret_control::srv::SetState>(
                topic_prefix_ + "/set_state",
                std::bind(&TurretROS2Node::setStateCallback, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Set state service created successfully: /%s/set_state", topic_prefix_.c_str());

            zero_yaw_service_ = this->create_service<std_srvs::srv::Trigger>(
                topic_prefix_ + "/zero_yaw",
                std::bind(&TurretROS2Node::zeroYawCallback, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Zero yaw service created: /%s/zero_yaw", topic_prefix_.c_str());

            // Test services (uncomment if needed for debugging)
            // test_service_ = this->create_service<std_srvs::srv::Empty>(
            //     topic_prefix_ + "/test",
            //     std::bind(&TurretROS2Node::testServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
            // trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            //     topic_prefix_ + "/trigger_test",
            //     std::bind(&TurretROS2Node::triggerServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create services: %s", e.what());
            throw;
        }
        
        // Timers
        heartbeat_timer_ = this->create_wall_timer(
            16667us, // 60Hz = 16.667ms
            std::bind(&TurretROS2Node::heartbeatCallback, this));
        
        update_timer_ = this->create_wall_timer(
            10ms, // 100Hz update rate
            std::bind(&TurretROS2Node::updateCallback, this));
        
        state_timer_ = this->create_wall_timer(
            50ms, // 20Hz state publishing
            std::bind(&TurretROS2Node::publishState, this));
            
        // Debug timer for periodic state logging (uncomment if needed)
        // debug_timer_ = this->create_wall_timer(
        //     5s, // Every 5 seconds
        //     std::bind(&TurretROS2Node::debugLog, this));

        RCLCPP_INFO(this->get_logger(), "Turret ROS2 node initialized successfully");
    }

    ~TurretROS2Node()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down turret node");
        // pigpio is terminated in main() after all nodes are destroyed
    }

private:
    enum class TurretState : uint8_t {
        IDLE = 0,
        READY = 1,
        RUNNING = 2,
        TELEOP = 3,
        TELEOP_ZERO = 4,
        DOCKING = 5
    };

    // Core components
    std::unique_ptr<Turret> turret_;
    YAML::Node config_;
    int turret_id_;
    std::string topic_prefix_;
    
    // State machine variables
    TurretState current_state_;
    bool is_zeroed_;
    bool is_zeroing_;
    double current_extension_;
    double current_velocity_;
    double desired_length_;
    double desired_velocity_;
    double desired_pitch_angle_;
    bool hold_current_pitch_;
    double zero_velocity_;
    std::future<bool> zero_future_;

    // Teleop zero state tracking
    bool sz_zeroed_ = false;       // Spiral zipper zeroed
    bool pitch_zeroed_ = false;    // Pitch encoder zeroed
    bool yaw_zeroed_ = false;      // Yaw motor zeroed
    double current_pitch_angle_ = 0.0;  // Current pitch angle from encoder
    double current_yaw_angle_ = 0.0;    // Current yaw angle from motor feedback
    bool docking_enabled_ = false;
    bool vision_tag_visible_ = false;
    bool have_camera_estimate_ = false;
    double docking_standoff_m_ = 0.30;
    double docking_extension_max_m_ = 0.90;
    double docking_pitch_limit_rad_ = 60.0 * M_PI / 180.0;
    double docking_pitch_hold_cap_rad_ = 50.0 * M_PI / 180.0;
    double docking_command_velocity_ = 1.5;
    double docking_stage_extension_m_ = 0.05;
    double docking_stage_pitch_rad_ = 25.0 * M_PI / 180.0;
    double docking_stage_pitch_tolerance_rad_ = 3.0 * M_PI / 180.0;
    double docking_stage_min_pitch_rad_ = 18.0 * M_PI / 180.0;
    double docking_stage_extension_tolerance_m_ = 0.005;
    bool docking_stage_active_ = false;
    double docking_camera_filter_alpha_ = 0.2;
    double docking_pose_timeout_sec_ = 0.5;
    double docking_lateral_tolerance_m_ = 0.05;
    bool docking_hold_on_lateral_error_ = false;
    bool docking_lock_goal_camera_y_on_stage_complete_ = true;
    double docking_goal_camera_y_offset_m_ = -0.10;
    bool docking_stop_on_close_tracking_loss_ = true;
    double docking_tracking_loss_stop_distance_m_ = 0.25;
    double docking_camera_forward_sign_ = 1.0;
    double docking_camera_vertical_sign_ = 1.0;
    double docking_visual_extension_kp_ = 0.8;
    double docking_visual_pitch_kp_ = 10.0;
    double docking_visual_max_extension_rate_mps_ = 0.12;
    double docking_visual_max_pitch_rate_radps_ = 35.0 * M_PI / 180.0;
    double docking_visual_distance_deadband_m_ = 0.01;
    double docking_visual_vertical_deadband_m_ = 0.01;
    double docking_visual_command_period_sec_ = 0.25;
    double docking_visual_pitch_command_horizon_sec_ = 0.25;
    double docking_visual_extension_step_max_m_ = 0.08;
    double docking_visual_pitch_step_max_rad_ = 8.0 * M_PI / 180.0;
    double docking_visual_slowdown_start_m_ = 0.30;
    double docking_visual_near_goal_scale_ = 0.25;
    double docking_visual_vertical_priority_error_m_ = 0.02;
    double docking_visual_pitch_limit_guard_rad_ = 15.0 * M_PI / 180.0;
    bool docking_stopped_on_tracking_loss_ = false;
    double estimated_camera_x_base_ = 0.0;
    double estimated_camera_y_base_ = 0.0;
    double docking_goal_camera_y_m_ = 0.0;
    geometry_msgs::msg::PoseStamped latest_tag_pose_camera_;
    rclcpp::Time latest_tag_pose_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time docking_last_servo_update_{0, 0, RCL_ROS_TIME};
    
    
    // Cached load cell data received from LoadCellNode (updated via subscription)
    double latest_lc_force_{0.0};
    bool latest_lc_ready_{false};

    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
    rclcpp::Publisher<turret_control::msg::TurretState>::SharedPtr state_pub_;
    rclcpp::Publisher<turret_control::msg::TurretVelocities>::SharedPtr velocities_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr docking_camera_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr docking_target_pub_;

    // ROS2 subscribers
    rclcpp::Subscription<turret_control::msg::ZipperCommand>::SharedPtr zipper_cmd_sub_;
    rclcpp::Subscription<turret_control::msg::TurretTeleopCommand>::SharedPtr teleop_cmd_sub_;
    rclcpp::Subscription<turret_control::msg::LoadCellForce>::SharedPtr load_cell_force_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_tag_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_tag_visible_sub_;

    // Teleop command variables
    double teleop_sz_velocity_ = 0.0;
    double teleop_pitch_velocity_ = 0.0;
    double teleop_yaw_velocity_ = 0.0;

    // ROS2 services
    rclcpp::Service<turret_control::srv::ZeroTurret>::SharedPtr zero_service_;
    rclcpp::Service<turret_control::srv::SetState>::SharedPtr set_state_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_yaw_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr debug_timer_;

    void heartbeatCallback()
    {
        auto msg = std_msgs::msg::Empty();
        heartbeat_pub_->publish(msg);
        // Remove this debug line after testing - it will be too verbose
        // RCLCPP_DEBUG(this->get_logger(), "Heartbeat callback executed");
    }

    void updateCallback()
    {
        if (!turret_) return;
        
        try {
            turret_->Update();
            
            // Update current state from hardware
            updateCurrentState();
            
            // Execute state machine logic
            executeStateMachine();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in update callback: %s", e.what());
        }
    }

    void updateCurrentState()
    {
        // Get current extension from spiral zipper encoder
        if (turret_) {
            current_extension_ = turret_->GetSpiralZipperExtension();
            current_velocity_ = turret_->GetSpiralZipperVelocity();

            // Get pitch angle from encoder (only valid after zeroing)
            if (pitch_zeroed_) {
                current_pitch_angle_ = turret_->GetPitchAngle();
            }

            // Get yaw angle from motor feedback (only valid after zeroing)
            if (yaw_zeroed_) {
                current_yaw_angle_ = turret_->GetYawAngle();
            }

            // Reset stale pitch-zeroed UI state unless zeroing has completed or switch indicates zero.
            if (!is_zeroed_ && !is_zeroing_) {
            if (pitch_zeroed_ && !turret_->IsTurretLimitPressed()) {
                pitch_zeroed_ = false;
            }
        }
        }
    }

    void executeStateMachine()
    {
        // Handle non-blocking zeroing process
        if (is_zeroing_) {
            processZeroingSequence();
            return; // Don't process other states while zeroing
        }
        
        switch (current_state_) {
            case TurretState::IDLE:
                // In IDLE state, ensure zipper is stopped
                ensureMotorStopped();
                break;
                
            case TurretState::READY:
                // In READY state, zipper is zeroed but not moving
                ensureMotorStopped();
                break;
                
            case TurretState::RUNNING:
                // In RUNNING state, execute zipper commands
                if (is_zeroed_) {
                    executeZipperCommand();
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "In RUNNING state but not zeroed - cannot execute commands");
                }
                break;

            case TurretState::DOCKING:
                if (is_zeroed_) {
                    executeZipperCommand();
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "In DOCKING state but not zeroed - cannot execute commands");
                }
                break;

            case TurretState::TELEOP:
                // In TELEOP state, execute direct velocity commands (no zeroing required)
                executeTeleopCommand();
                break;

            case TurretState::TELEOP_ZERO:
                // In TELEOP_ZERO state, execute velocity commands while monitoring limit switches
                executeTeleopZeroCommand();
                break;
        }
    }
    
    void processZeroingSequence()
    {
        // Check if the async zeroing operation is complete
        if (zero_future_.valid()) {
            auto status = zero_future_.wait_for(std::chrono::milliseconds(0));
            if (status == std::future_status::ready) {
                // Zeroing is complete
                bool success = zero_future_.get();
                is_zeroing_ = false;
                
                if (success) {
                    is_zeroed_ = true;
                    sz_zeroed_ = true;
                    pitch_zeroed_ = true;
                    current_state_ = TurretState::READY;
                    current_extension_ = turret_->GetSpiralZipperExtension();
                    current_velocity_ = 0.0;
                    current_pitch_angle_ = turret_->GetPitchAngle();
                    desired_length_ = current_extension_;
                    desired_velocity_ = 0.0;
                    desired_pitch_angle_ = current_pitch_angle_;
                    hold_current_pitch_ = true;
                    RCLCPP_INFO(this->get_logger(), "Turret zeroing completed successfully! State changed to READY");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Turret zeroing failed! Returning to IDLE state");
                    is_zeroed_ = false;
                    sz_zeroed_ = false;
                    pitch_zeroed_ = false;
                    current_state_ = TurretState::IDLE;
                }
            } else {
                // Still zeroing - log progress occasionally
                static int counter = 0;
                if (++counter % 500 == 0) {  // Log every ~5 seconds (at 100Hz update rate)
                    RCLCPP_INFO(this->get_logger(), "Zeroing in progress...");
                }
            }
        }
    }

    void executeZipperCommand()
    {
        // Command the spiral zipper to move to desired position with desired velocity
        try {
            if (turret_) {
                if (current_state_ == TurretState::DOCKING) {
                    updateDockingSetpoint();
                }
                // desired_pitch_angle_ is frozen when the command is accepted.
                double desired_pitch = desired_pitch_angle_;
                double max_velocity = std::abs(desired_velocity_);
                turret_->ActuateTurretCable(
                    static_cast<float>(desired_length_),
                    static_cast<float>(desired_pitch),
                    static_cast<float>(max_velocity),
                    hold_current_pitch_,
                    current_state_ == TurretState::DOCKING);

    // RCLCPP_INFO(this->get_logger(),
    //     "EXECUTING zipper command: length=%.3f meters, max_velocity=%.3f",
    //     desired_length_, max_velocity);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error executing zipper command: %s", e.what());
        }
    }

    void executeTeleopCommand()
    {
        // Command motors directly with velocity commands
        try {
            if (turret_) {
                bool sz_limit_pressed = turret_->IsSpiralZipperLimitPressed();
                bool turret_limit_pressed = turret_->IsTurretLimitPressed();

                double effective_sz_velocity = teleop_sz_velocity_;
                double effective_pitch_velocity = teleop_pitch_velocity_;
                if (sz_limit_pressed && effective_sz_velocity < 0.0) {
                    effective_sz_velocity = 0.0;
                }
                if (turret_limit_pressed && effective_pitch_velocity < 0.0) {
                    effective_pitch_velocity = 0.0;
                }

                // Log motor commands when non-zero (throttled to avoid spam)
                if (std::abs(effective_sz_velocity) > 0.001 ||
                    std::abs(effective_pitch_velocity) > 0.001 ||
                    std::abs(teleop_yaw_velocity_) > 0.001) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "EXEC Teleop: Commanding motors sz=%.3f, pitch=%.3f, yaw=%.3f",
                        effective_sz_velocity, effective_pitch_velocity, teleop_yaw_velocity_);
                    if (sz_limit_pressed || turret_limit_pressed) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "LIMIT OVERRIDE active in TELEOP: sz_limit=%s turret_limit=%s (SZ/pitch only block negative motion)",
                            sz_limit_pressed ? "YES" : "NO",
                            turret_limit_pressed ? "YES" : "NO");
                    }
                }

                turret_->SetSpiralZipperVelocity(effective_sz_velocity);
                turret_->SetPitchVelocity(effective_pitch_velocity);
                turret_->SetYawVelocity(teleop_yaw_velocity_);

                // Publish velocity commands
                publishVelocities();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error executing teleop command: %s", e.what());
        }
    }

    void executeTeleopZeroCommand()
    {
        // In TELEOP_ZERO mode, execute velocity commands while monitoring for zeroing events
        try {
            if (!turret_) return;

            bool sz_limit_pressed = turret_->IsSpiralZipperLimitPressed();
            bool turret_limit_pressed = turret_->IsTurretLimitPressed();

            // Execute SZ and yaw commands continuously in zero mode.
            double effective_sz_velocity = sz_limit_pressed ? 0.0 : teleop_sz_velocity_;
            turret_->SetSpiralZipperVelocity(effective_sz_velocity);
            turret_->SetYawVelocity(teleop_yaw_velocity_);

            if (sz_limit_pressed && std::abs(teleop_sz_velocity_) > 0.001) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "TELEOP_ZERO: SZ limit switch pressed -> SZ command forced to 0.0");
            }

            // Publish velocity commands
            publishVelocities();

            // Monitor spiral zipper limit switch for zeroing
            // The SZ zeroing is detected when extension is near zero after retracting
            // For now, we check if extension is very small (near limit)

            // DEBUG: Log SZ zeroing conditions every 2 seconds
            if (!sz_zeroed_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "TELEOP_ZERO: SZ zeroing - extension=%.4f, sz_vel=%.3f",
                    current_extension_, teleop_sz_velocity_);
            }

            if (!sz_zeroed_ && current_extension_ < 0.001 && teleop_sz_velocity_ < 0) {
                // SZ has been retracted to limit - zero the encoder
                turret_->ZeroSpiralZipper();  // This will reset the encoder
                sz_zeroed_ = true;
                RCLCPP_INFO(this->get_logger(), "TELEOP_ZERO: Spiral zipper zeroed");
            }

            // Monitor turret limit switch for zeroing (used to zero pitch encoder)

            // DEBUG: Log pitch zeroing conditions every 2 seconds
            if (!pitch_zeroed_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "TELEOP_ZERO: Pitch zeroing (turret switch) - limit_pressed=%s, pitch_vel=%.3f",
                    turret_limit_pressed ? "YES" : "NO", teleop_pitch_velocity_);
            }

            double effective_pitch_velocity = turret_limit_pressed ? 0.0 : teleop_pitch_velocity_;
            if (!pitch_zeroed_ && turret_limit_pressed) {
                // Stop pitch whenever turret limit switch is pressed.
                turret_->SetPitchVelocity(effective_pitch_velocity);

                // Only accept pitch zero when SZ has already completed zeroing.
                if (sz_zeroed_) {
                    turret_->ZeroPitchEncoder();
                    pitch_zeroed_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                 "TELEOP_ZERO: SZ is zeroed and turret switch pressed, pitch encoder zeroed");
                }
            } else {
                // If switch is not pressed, keep following the requested pitch command.
                // This also re-activates motion after a release during zeroing.
                turret_->SetPitchVelocity(effective_pitch_velocity);
            }

            // Yaw zeroing is triggered manually by user when in desired position
            // (handled in teleopCommandCallback with a special flag or service)

            // Check if all components are zeroed
            if (sz_zeroed_ && pitch_zeroed_ && yaw_zeroed_) {
                is_zeroed_ = true;
                RCLCPP_INFO(this->get_logger(), "TELEOP_ZERO: All components zeroed!");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error executing teleop zero command: %s", e.what());
        }
    }

    void publishVelocities()
    {
        // Publish commanded velocities to /cmd_vel/turret{id}/velocities
        auto vel_msg = turret_control::msg::TurretVelocities();
        vel_msg.spiral_zipper_velocity = teleop_sz_velocity_;
        vel_msg.pitch_velocity = teleop_pitch_velocity_;
        vel_msg.yaw_velocity = teleop_yaw_velocity_;
        velocities_pub_->publish(vel_msg);
    }

    void publishState()
    {
        auto state_msg = turret_control::msg::TurretState();
        state_msg.state = static_cast<uint8_t>(current_state_);
        state_msg.extension_length = current_extension_;
        state_msg.velocity = current_velocity_;
        state_msg.pitch_angle = current_pitch_angle_;
        state_msg.yaw_angle = current_yaw_angle_;
        state_msg.is_zeroed = is_zeroed_;
        state_msg.sz_zeroed = sz_zeroed_;
        state_msg.pitch_zeroed = pitch_zeroed_;
        state_msg.yaw_zeroed = yaw_zeroed_;
        // Force and readiness come from the separate LoadCellNode via subscription
        state_msg.load_cell_ready = latest_lc_ready_;
        state_msg.force = latest_lc_force_;

        switch (current_state_) {
            case TurretState::IDLE:
                state_msg.status_message = "Turret is in IDLE state";
                break;
            case TurretState::READY:
                state_msg.status_message = is_zeroed_ ? "Turret is READY" : "Turret needs zeroing";
                break;
            case TurretState::RUNNING:
                state_msg.status_message = "Turret is RUNNING";
                break;
            case TurretState::DOCKING:
                state_msg.status_message = "Turret is DOCKING";
                break;
            case TurretState::TELEOP:
                state_msg.status_message = "Turret is in TELEOP mode";
                break;
            case TurretState::TELEOP_ZERO:
                {
                    std::string zero_status = "TELEOP_ZERO: ";
                    zero_status += sz_zeroed_ ? "SZ[OK] " : "SZ[--] ";
                    zero_status += pitch_zeroed_ ? "Pitch[OK] " : "Pitch[--] ";
                    zero_status += yaw_zeroed_ ? "Yaw[OK]" : "Yaw[--]";
                    state_msg.status_message = zero_status;
                }
                break;
        }

        state_pub_->publish(state_msg);
    }
    
    // Receives force data published by LoadCellNode and caches it for the state message
    void loadCellForceCallback(const turret_control::msg::LoadCellForce::SharedPtr msg)
    {
        latest_lc_force_ = msg->force;
        latest_lc_ready_ = msg->calibrated;
    }

    void visionTagPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_tag_pose_camera_ = *msg;
        latest_tag_pose_time_ = this->now();
    }

    void visionTagVisibleCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        vision_tag_visible_ = msg->data;
    }

    void updateDockingSetpoint()
    {
        if (!turret_ || !pitch_zeroed_) {
            return;
        }

        if (docking_stage_active_) {
            const bool stage_extension_reached =
                current_extension_ >= (docking_stage_extension_m_ - docking_stage_extension_tolerance_m_);
            const bool stage_pitch_reached =
                current_pitch_angle_ >= (docking_stage_pitch_rad_ - docking_stage_pitch_tolerance_rad_);
            const bool stage_pitch_visible_ready =
                vision_tag_visible_ && current_pitch_angle_ >= docking_stage_min_pitch_rad_;
            if (!stage_extension_reached || (!stage_pitch_reached && !stage_pitch_visible_ready)) {
                desired_length_ = std::max(current_extension_, docking_stage_extension_m_);
                desired_pitch_angle_ = docking_stage_pitch_rad_;
                desired_velocity_ = std::max(docking_command_velocity_, 1.5);
                hold_current_pitch_ = false;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Docking stage move: cmd_ext=%.3f m cmd_pitch=%.2f deg current_ext=%.3f m current_pitch=%.2f deg",
                    desired_length_, desired_pitch_angle_ * 180.0 / M_PI,
                    current_extension_, current_pitch_angle_ * 180.0 / M_PI);
                return;
            }

            docking_stage_active_ = false;
            have_camera_estimate_ = false;
            docking_stopped_on_tracking_loss_ = false;
            const double stage_camera_y =
                (docking_lock_goal_camera_y_on_stage_complete_ && vision_tag_visible_)
                    ? latest_tag_pose_camera_.pose.position.y
                    : 0.0;
            docking_goal_camera_y_m_ = stage_camera_y + docking_goal_camera_y_offset_m_;
            docking_last_servo_update_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
            RCLCPP_INFO(this->get_logger(),
                "Docking stage complete at ext=%.3f m pitch=%.2f deg (tag_visible=%s, stage_cam_y=%.3f m, goal_cam_y=%.3f m) - switching to vision-guided docking",
                current_extension_, current_pitch_angle_ * 180.0 / M_PI,
                vision_tag_visible_ ? "true" : "false",
                stage_camera_y,
                docking_goal_camera_y_m_);
        }

        const double pose_age = (this->now() - latest_tag_pose_time_).seconds();
        const bool close_to_camera =
            latest_tag_pose_time_.nanoseconds() != 0 &&
            latest_tag_pose_camera_.pose.position.z <= docking_tracking_loss_stop_distance_m_;
        if (docking_stopped_on_tracking_loss_) {
            desired_length_ = current_extension_;
            desired_pitch_angle_ = current_pitch_angle_;
            desired_velocity_ = 0.0;
            hold_current_pitch_ = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Docking is latched stopped after close-range tracking loss. Re-arm DOCKING to continue.");
            return;
        }
        if (!vision_tag_visible_ || pose_age > docking_pose_timeout_sec_) {
            desired_length_ = current_extension_;
            desired_pitch_angle_ = current_pitch_angle_;
            desired_velocity_ = 0.0;
            hold_current_pitch_ = true;
            if (docking_stop_on_close_tracking_loss_ && close_to_camera) {
                docking_stopped_on_tracking_loss_ = true;
                RCLCPP_WARN(this->get_logger(),
                    "Docking stopped due to close-range tracking loss (visible=%s age=%.3f s last_z=%.3f m <= %.3f m).",
                    vision_tag_visible_ ? "true" : "false",
                    pose_age,
                    latest_tag_pose_camera_.pose.position.z,
                    docking_tracking_loss_stop_distance_m_);
                return;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Docking active but no fresh vision pose available (visible=%s age=%.3f s)",
                vision_tag_visible_ ? "true" : "false", pose_age);
            return;
        }

        const double lateral_error = latest_tag_pose_camera_.pose.position.x;
        if (std::fabs(lateral_error) > docking_lateral_tolerance_m_) {
            if (docking_hold_on_lateral_error_) {
                desired_length_ = current_extension_;
                desired_pitch_angle_ = current_pitch_angle_;
                desired_velocity_ = 0.0;
                hold_current_pitch_ = true;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Docking lateral camera error %.3f m exceeds tolerance %.3f m. Holding position until yaw is aligned.",
                    lateral_error, docking_lateral_tolerance_m_);
                return;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Docking lateral camera error %.3f m exceeds tolerance %.3f m, but continuing because lateral hold is disabled.",
                lateral_error, docking_lateral_tolerance_m_);
        }

        const rclcpp::Time servo_now = this->now();
        double servo_dt = docking_visual_command_period_sec_;
        if (docking_last_servo_update_.nanoseconds() != 0) {
            servo_dt = (servo_now - docking_last_servo_update_).seconds();
        }
        if (docking_last_servo_update_.nanoseconds() != 0 &&
            servo_dt < docking_visual_command_period_sec_) {
            return;
        }
        docking_last_servo_update_ = servo_now;
        servo_dt = std::clamp(servo_dt, docking_visual_command_period_sec_, docking_visual_command_period_sec_ * 2.0);

        const double z_cam = latest_tag_pose_camera_.pose.position.z;
        const double y_cam = latest_tag_pose_camera_.pose.position.y;
        const double z_error = z_cam - docking_standoff_m_;
        const double y_error = docking_goal_camera_y_m_ - y_cam;

        double extension_rate_cmd = 0.0;
        if (std::fabs(z_error) > docking_visual_distance_deadband_m_) {
            extension_rate_cmd = docking_visual_extension_kp_ * z_error;
        }
        extension_rate_cmd = std::clamp(
            extension_rate_cmd,
            -docking_visual_max_extension_rate_mps_,
            docking_visual_max_extension_rate_mps_);

        double pitch_rate_cmd = 0.0;
        if (std::fabs(y_error) > docking_visual_vertical_deadband_m_) {
            pitch_rate_cmd = docking_visual_pitch_kp_ * y_error;
        }
        pitch_rate_cmd = std::clamp(
            pitch_rate_cmd,
            -docking_visual_max_pitch_rate_radps_,
            docking_visual_max_pitch_rate_radps_);

        const bool prioritize_extension_over_pitch =
            z_error > 0.05 && std::fabs(y_error) <= docking_visual_vertical_priority_error_m_;
        if (prioritize_extension_over_pitch) {
            pitch_rate_cmd = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Docking: vertical error %.3f m is small while depth error %.3f m remains; holding pitch and prioritizing extension.",
                y_error, z_error);
        }

        double near_goal_scale = 1.0;
        if (docking_visual_slowdown_start_m_ > docking_standoff_m_ && z_cam < docking_visual_slowdown_start_m_) {
            const double normalized = std::clamp(
                (z_cam - docking_standoff_m_) /
                std::max(1e-6, docking_visual_slowdown_start_m_ - docking_standoff_m_),
                0.0,
                1.0);
            near_goal_scale =
                docking_visual_near_goal_scale_ +
                (1.0 - docking_visual_near_goal_scale_) * normalized;
        }

        double extension_scale = near_goal_scale;
        if (z_cam < docking_visual_slowdown_start_m_ &&
            std::fabs(y_error) > docking_visual_vertical_priority_error_m_) {
            extension_scale *= std::clamp(
                docking_visual_vertical_priority_error_m_ / std::fabs(y_error),
                0.0,
                1.0);
        }

        const bool near_positive_pitch_limit =
            current_pitch_angle_ >= (docking_pitch_limit_rad_ - docking_visual_pitch_limit_guard_rad_);
        const bool near_negative_pitch_limit =
            current_pitch_angle_ <= (-docking_pitch_limit_rad_ + docking_visual_pitch_limit_guard_rad_);
        double pitch_limit_scale = 1.0;
        if (pitch_rate_cmd > 0.0) {
            const double remaining_margin = docking_pitch_limit_rad_ - current_pitch_angle_;
            pitch_limit_scale = std::clamp(
                remaining_margin / std::max(1e-6, docking_visual_pitch_limit_guard_rad_),
                0.0,
                1.0);
        } else if (pitch_rate_cmd < 0.0) {
            const double remaining_margin = current_pitch_angle_ + docking_pitch_limit_rad_;
            pitch_limit_scale = std::clamp(
                remaining_margin / std::max(1e-6, docking_visual_pitch_limit_guard_rad_),
                0.0,
                1.0);
        }
        if (pitch_rate_cmd > 0.0 && current_pitch_angle_ >= docking_pitch_hold_cap_rad_) {
            pitch_rate_cmd = 0.0;
            pitch_limit_scale = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Docking: holding pitch at configured cap %.1f deg while continuing extension.",
                docking_pitch_hold_cap_rad_ * 180.0 / M_PI);
        }
        if ((near_positive_pitch_limit && pitch_rate_cmd > 0.0) ||
            (near_negative_pitch_limit && pitch_rate_cmd < 0.0)) {
            if (std::fabs(y_error) > docking_visual_vertical_priority_error_m_) {
                extension_scale = 0.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Docking: pitch is near its limit with unresolved vertical error; pausing extension to preserve tag visibility.");
            } else {
                pitch_rate_cmd = 0.0;
                pitch_limit_scale = 0.0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Docking: pitch is near its limit but vertical error is small; holding pitch and continuing extension.");
            }
        }

        extension_rate_cmd *= extension_scale;
        pitch_rate_cmd *= (0.6 + 0.4 * near_goal_scale);
        pitch_rate_cmd *= pitch_limit_scale;

        double extension_step_limit = docking_visual_extension_step_max_m_;
        if (z_cam < docking_visual_slowdown_start_m_) {
            extension_step_limit *= near_goal_scale;
        }
        extension_step_limit = std::clamp(
            extension_step_limit,
            docking_visual_distance_deadband_m_,
            docking_visual_extension_step_max_m_);
        extension_step_limit = std::min(extension_step_limit, std::max(docking_visual_distance_deadband_m_, std::fabs(z_error)));

        double pitch_step_limit = docking_visual_pitch_step_max_rad_;
        if (z_cam < docking_visual_slowdown_start_m_) {
            pitch_step_limit *= (0.5 + 0.5 * near_goal_scale);
        }
        pitch_step_limit *= std::max(0.25, pitch_limit_scale);
        pitch_step_limit = std::clamp(
            pitch_step_limit,
            1.0 * M_PI / 180.0,
            docking_visual_pitch_step_max_rad_);

        const double extension_step = std::clamp(
            extension_rate_cmd * servo_dt,
            -extension_step_limit,
            extension_step_limit);
        const double pitch_dt = std::clamp(
            docking_visual_pitch_command_horizon_sec_,
            0.05,
            servo_dt);
        const double pitch_step = std::clamp(
            pitch_rate_cmd * pitch_dt,
            -pitch_step_limit,
            pitch_step_limit);

        desired_length_ = std::clamp(
            current_extension_ + extension_step,
            0.0,
            docking_extension_max_m_);
        desired_pitch_angle_ = std::clamp(
            current_pitch_angle_ + pitch_step,
            -docking_pitch_limit_rad_,
            docking_pitch_limit_rad_);
        desired_pitch_angle_ = std::min(desired_pitch_angle_, docking_pitch_hold_cap_rad_);
        desired_velocity_ = std::max(docking_command_velocity_, 1.5);
        hold_current_pitch_ = false;

        auto camera_pose_msg = geometry_msgs::msg::PoseStamped();
        camera_pose_msg.header.stamp = this->now();
        camera_pose_msg.header.frame_id = latest_tag_pose_camera_.header.frame_id.empty()
            ? "camera_link" : latest_tag_pose_camera_.header.frame_id;
        camera_pose_msg.pose.position.x = z_cam;
        camera_pose_msg.pose.position.y = 0.0;
        camera_pose_msg.pose.position.z = y_cam;
        camera_pose_msg.pose.orientation.w = 1.0;
        docking_camera_pub_->publish(camera_pose_msg);

        auto target_pose_msg = geometry_msgs::msg::PoseStamped();
        target_pose_msg.header = camera_pose_msg.header;
        target_pose_msg.pose.position.x = docking_standoff_m_;
        target_pose_msg.pose.position.y = 0.0;
        target_pose_msg.pose.position.z = docking_goal_camera_y_m_;
        target_pose_msg.pose.orientation.w = 1.0;
        docking_target_pub_->publish(target_pose_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Docking servo: cam_yz=(%.3f, %.3f) m target_yz=(%.3f, %.3f) m err_yz=(%.3f, %.3f) scale=%.2f step=[ext=%.3f m pitch=%.2f deg] rates=[ext=%.3f m/s pitch=%.2f deg/s] cmd=[ext=%.3f m pitch=%.2f deg] cam_x=%.3f m",
            y_cam, z_cam,
            docking_goal_camera_y_m_, docking_standoff_m_,
            y_error, z_error,
            extension_scale,
            extension_step, pitch_step * 180.0 / M_PI,
            extension_rate_cmd, pitch_rate_cmd * 180.0 / M_PI,
            desired_length_, desired_pitch_angle_ * 180.0 / M_PI,
            lateral_error);
    }


    void debugLog()
    {
        std::string status = is_zeroing_ ? " [ZEROING]" : "";
        RCLCPP_INFO(this->get_logger(), 
            "DEBUG - State: %d, Zeroed: %s, Extension: %.3f, Velocity: %.3f%s", 
            static_cast<int>(current_state_),
            is_zeroed_ ? "true" : "false",
            current_extension_,
            current_velocity_,
            status.c_str());
    }
    
    void stopMotor()
    {
        if (turret_) {
            turret_->StopSpiralZipper();
            RCLCPP_INFO(this->get_logger(), "Motor stopped");
        }
    }
    
    void ensureMotorStopped()
    {
        // Only stop if motor is actually moving to avoid spam
        if (turret_ && std::abs(current_velocity_) > 0.01) {
            turret_->StopSpiralZipper();
        }
    }
    
    // void testServiceCallback(
    //     const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    //     std::shared_ptr<std_srvs::srv::Empty::Response> response)
    // {
    //     (void)request; // Unused parameter
    //     (void)response; // Unused parameter
    //     RCLCPP_INFO(this->get_logger(), "TEST SERVICE CALLED! Service communication is working!");
    // }
    
    // void triggerServiceCallback(
    //     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    //     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    // {
    //     (void)request; // Unused parameter
    //     response->success = true;
    //     response->message = "Trigger service called successfully!";
    //     RCLCPP_INFO(this->get_logger(), "TRIGGER SERVICE CALLED! Response: %s", response->message.c_str());
    // }

    void zipperCommandCallback(const turret_control::msg::ZipperCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "ZIPPER COMMAND RECEIVED: length=%.3f, velocity=%.3f, desired_pitch=%.3f rad, hold_pitch=%s",
            msg->desired_length, msg->desired_velocity, msg->desired_pitch_angle,
            msg->hold_current_pitch ? "true" : "false");

        if (current_state_ != TurretState::RUNNING) {
            RCLCPP_WARN(this->get_logger(),
                "Received zipper command but turret is not in RUNNING state (current: %d)",
                static_cast<int>(current_state_));
            return;
        }

        if (!is_zeroed_) {
            RCLCPP_WARN(this->get_logger(),
                "Received zipper command but turret is not zeroed");
            return;
        }

        desired_length_ = msg->desired_length;
        desired_velocity_ = msg->desired_velocity;
        desired_pitch_angle_ = msg->desired_pitch_angle;
        hold_current_pitch_ = msg->hold_current_pitch;
        if (!hold_current_pitch_ && std::abs(desired_pitch_angle_) > (2.0 * M_PI)) {
            RCLCPP_WARN(this->get_logger(),
                "Requested pitch %.3f rad exceeds one full turn. This interface expects radians; 10 deg should be 0.1745 rad.",
                desired_pitch_angle_);
        }
        // Backward compatibility with older publishers that only send length/velocity:
        // treat missing/zero pitch input as "hold current pitch".
        if (!hold_current_pitch_ && std::abs(desired_pitch_angle_) < 1e-9) {
            hold_current_pitch_ = true;
        }
        if (hold_current_pitch_) {
            desired_pitch_angle_ = current_pitch_angle_;
        }

        RCLCPP_INFO(this->get_logger(),
            "Zipper command accepted: length=%.3f, velocity=%.3f, desired_pitch=%.3f rad, hold_pitch=%s",
            desired_length_, desired_velocity_, desired_pitch_angle_, hold_current_pitch_ ? "true" : "false");
    }

    void teleopCommandCallback(const turret_control::msg::TurretTeleopCommand::SharedPtr msg)
    {
        if (current_state_ != TurretState::TELEOP && current_state_ != TurretState::TELEOP_ZERO) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Received teleop command but turret is not in TELEOP/TELEOP_ZERO state (current: %d)",
                static_cast<int>(current_state_));
            return;
        }

        // TELEOP/TELEOP_ZERO mode: Accept velocity commands
        teleop_sz_velocity_ = msg->spiral_zipper_velocity;
        teleop_pitch_velocity_ = msg->pitch_velocity;
        teleop_yaw_velocity_ = msg->yaw_velocity;

        // Log all non-zero commands at INFO level for debugging
        if (std::abs(teleop_sz_velocity_) > 0.001 || std::abs(teleop_pitch_velocity_) > 0.001 || std::abs(teleop_yaw_velocity_) > 0.001) {
            RCLCPP_INFO(this->get_logger(),
                "RX Teleop CMD: sz=%.3f, pitch=%.3f, yaw=%.3f",
                teleop_sz_velocity_, teleop_pitch_velocity_, teleop_yaw_velocity_);
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                "Teleop command: sz_vel=%.3f, pitch_vel=%.3f, yaw_vel=%.3f",
                teleop_sz_velocity_, teleop_pitch_velocity_, teleop_yaw_velocity_);
        }
    }

    void zeroTurretCallback(
        const std::shared_ptr<turret_control::srv::ZeroTurret::Request> request,
        std::shared_ptr<turret_control::srv::ZeroTurret::Response> response)
    {
        (void)request; // Unused parameter
        
        // Get velocity from parameter (can be set via ros2 param set)
        double requested_velocity = this->get_parameter("zero_velocity").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Zero service called with velocity %.2f rad/s! Current state: %d", 
                    requested_velocity, static_cast<int>(current_state_));
        
        if (current_state_ != TurretState::IDLE) {
            response->success = false;
            response->message = "Cannot zero turret: must be in IDLE state (current: " + std::to_string(static_cast<int>(current_state_)) + ")";
            RCLCPP_WARN(this->get_logger(), "Zero request denied: turret not in IDLE state (current: %d)", static_cast<int>(current_state_));
            return;
        }
        
        // Clamp and validate the velocity:
        //  - enforce retract direction (negative)
        //  - allow low speeds, keep bounded for safety
        if (requested_velocity > 0) {
            requested_velocity = -requested_velocity;
        }
        const double kMinZeroVelocity = 0.005;
        const double kMaxZeroVelocity = 3.0;
        if (requested_velocity > 0 || std::fabs(requested_velocity) < kMinZeroVelocity) {
            response->success = false;
            response->message = "Invalid velocity: must be negative and magnitude >= "
                                + std::to_string(kMinZeroVelocity) + " rad/s";
            RCLCPP_WARN(this->get_logger(), "Zero request denied: unsafe velocity %.2f", requested_velocity);
            return;
        }

        if (std::abs(requested_velocity) > kMaxZeroVelocity) {
            requested_velocity = -kMaxZeroVelocity;
            RCLCPP_WARN(this->get_logger(),
                "Requested zero velocity capped to %.2f rad/s for safety", requested_velocity);
        }
        RCLCPP_INFO(this->get_logger(), "Zeroing will use effective velocity: %.3f rad/s", requested_velocity);
        
        try {
            RCLCPP_INFO(this->get_logger(), "Starting turret zero sequence...");
            
            // Start zeroing in a separate thread to avoid blocking the executor
            is_zeroing_ = true;
            sz_zeroed_ = false;
            pitch_zeroed_ = false;
            zero_velocity_ = requested_velocity;  // Use requested velocity
            
            zero_future_ = std::async(std::launch::async, [this]() -> bool {
                try {
                    return turret_->ZeroTurret(zero_velocity_);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Zeroing failed: %s", e.what());
                    return false;
                }
            });
            
            response->success = true;
            response->message = "Turret zeroing started (async)";
            RCLCPP_INFO(this->get_logger(), "Turret zeroing started at %.2f rad/s", zero_velocity_);
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to zero turret: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Failed to zero turret: %s", e.what());
        }
    }

    void setStateCallback(
        const std::shared_ptr<turret_control::srv::SetState::Request> request,
        std::shared_ptr<turret_control::srv::SetState::Response> response)
    {
        auto requested_state = static_cast<TurretState>(request->desired_state);

        response->current_state = static_cast<uint8_t>(current_state_);

        // If leaving TELEOP or TELEOP_ZERO state, exit MIT mode for motors
        if ((current_state_ == TurretState::TELEOP || current_state_ == TurretState::TELEOP_ZERO) &&
            requested_state != TurretState::TELEOP && requested_state != TurretState::TELEOP_ZERO) {
            if (turret_) {
                turret_->ExitTeleopMode();
            }
        }

        // State transition logic
        switch (requested_state) {
            case TurretState::IDLE:
                // Stop motor when going to IDLE
                stopMotor();
                docking_enabled_ = false;
                docking_stage_active_ = false;
                current_state_ = TurretState::IDLE;
                response->success = true;
                response->message = "State changed to IDLE";
                break;
                
            case TurretState::READY:
                if (!is_zeroed_) {
                    response->success = false;
                    response->message = "Cannot go to READY: turret not zeroed";
                } else {
                    // Stop motor when going to READY
                    stopMotor();
                    docking_enabled_ = false;
                    docking_stage_active_ = false;
                    current_state_ = TurretState::READY;
                    response->success = true;
                    response->message = "State changed to READY";
                }
                break;
                
            case TurretState::RUNNING:
                if (!is_zeroed_) {
                    response->success = false;
                    response->message = "Cannot go to RUNNING: turret not zeroed";
                } else {
                    docking_enabled_ = false;
                    docking_stage_active_ = false;
                    current_state_ = TurretState::RUNNING;
                    response->success = true;
                    response->message = "State changed to RUNNING";
                }
                break;

            case TurretState::DOCKING:
                if (!is_zeroed_) {
                    response->success = false;
                    response->message = "Cannot go to DOCKING: turret not zeroed";
                } else {
                    docking_standoff_m_ = this->get_parameter("docking_standoff_m").as_double();
                    docking_extension_max_m_ = this->get_parameter("docking_extension_max_m").as_double();
                    docking_pitch_limit_rad_ = this->get_parameter("docking_pitch_limit_deg").as_double() * M_PI / 180.0;
                    docking_pitch_hold_cap_rad_ =
                        this->get_parameter("docking_pitch_hold_cap_deg").as_double() * M_PI / 180.0;
                    docking_pitch_hold_cap_rad_ = std::clamp(
                        docking_pitch_hold_cap_rad_,
                        0.0,
                        docking_pitch_limit_rad_);
                    docking_command_velocity_ = this->get_parameter("docking_command_velocity").as_double();
                    docking_stage_extension_m_ = this->get_parameter("docking_stage_extension_m").as_double();
                    docking_stage_pitch_rad_ = this->get_parameter("docking_stage_pitch_deg").as_double() * M_PI / 180.0;
                    docking_stage_pitch_tolerance_rad_ = this->get_parameter("docking_stage_pitch_tolerance_deg").as_double() * M_PI / 180.0;
                    docking_stage_min_pitch_rad_ = this->get_parameter("docking_stage_min_pitch_deg").as_double() * M_PI / 180.0;
                    docking_stage_extension_tolerance_m_ = this->get_parameter("docking_stage_extension_tolerance_m").as_double();
                    docking_camera_filter_alpha_ = this->get_parameter("docking_camera_filter_alpha").as_double();
                    docking_pose_timeout_sec_ = this->get_parameter("docking_pose_timeout_sec").as_double();
                    docking_lateral_tolerance_m_ = this->get_parameter("docking_lateral_tolerance_m").as_double();
                    docking_hold_on_lateral_error_ = this->get_parameter("docking_hold_on_lateral_error").as_bool();
                    docking_lock_goal_camera_y_on_stage_complete_ =
                        this->get_parameter("docking_lock_goal_camera_y_on_stage_complete").as_bool();
                    docking_goal_camera_y_offset_m_ =
                        this->get_parameter("docking_goal_camera_y_offset_m").as_double();
                    docking_stop_on_close_tracking_loss_ =
                        this->get_parameter("docking_stop_on_close_tracking_loss").as_bool();
                    docking_tracking_loss_stop_distance_m_ =
                        this->get_parameter("docking_tracking_loss_stop_distance_m").as_double();
                    docking_camera_forward_sign_ = this->get_parameter("docking_camera_forward_sign").as_double();
                    docking_camera_vertical_sign_ = this->get_parameter("docking_camera_vertical_sign").as_double();
                    docking_visual_extension_kp_ = this->get_parameter("docking_visual_extension_kp").as_double();
                    docking_visual_pitch_kp_ = this->get_parameter("docking_visual_pitch_kp").as_double();
                    docking_visual_max_extension_rate_mps_ =
                        this->get_parameter("docking_visual_max_extension_rate_mps").as_double();
                    docking_visual_max_pitch_rate_radps_ =
                        this->get_parameter("docking_visual_max_pitch_rate_deg_s").as_double() * M_PI / 180.0;
                    docking_visual_distance_deadband_m_ =
                        this->get_parameter("docking_visual_distance_deadband_m").as_double();
                    docking_visual_vertical_deadband_m_ =
                        this->get_parameter("docking_visual_vertical_deadband_m").as_double();
                    docking_visual_command_period_sec_ =
                        this->get_parameter("docking_visual_command_period_sec").as_double();
                    docking_visual_pitch_command_horizon_sec_ =
                        this->get_parameter("docking_visual_pitch_command_horizon_sec").as_double();
                    docking_visual_extension_step_max_m_ =
                        this->get_parameter("docking_visual_extension_step_max_m").as_double();
                    docking_visual_pitch_step_max_rad_ =
                        this->get_parameter("docking_visual_pitch_step_max_deg").as_double() * M_PI / 180.0;
                    docking_visual_slowdown_start_m_ =
                        this->get_parameter("docking_visual_slowdown_start_m").as_double();
                    docking_visual_near_goal_scale_ =
                        this->get_parameter("docking_visual_near_goal_scale").as_double();
                    docking_visual_vertical_priority_error_m_ =
                        this->get_parameter("docking_visual_vertical_priority_error_m").as_double();
                    docking_visual_pitch_limit_guard_rad_ =
                        this->get_parameter("docking_visual_pitch_limit_guard_deg").as_double() * M_PI / 180.0;
                    docking_enabled_ = true;
                    docking_stage_active_ = true;
                    docking_stopped_on_tracking_loss_ = false;
                    have_camera_estimate_ = false;
                    docking_goal_camera_y_m_ = 0.0;
                    docking_last_servo_update_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
                    current_state_ = TurretState::DOCKING;
                    response->success = true;
                    response->message = "State changed to DOCKING";
                    RCLCPP_INFO(this->get_logger(),
                        "Docking armed: standoff=%.3f m, extension_max=%.3f m, pitch_limit=%.1f deg, pitch_hold_cap=%.1f deg, stage=[ext=%.3f m pitch=%.1f deg min_pitch=%.1f deg], lateral_hold=%s lock_goal_cam_y=%s offset_y=%.3f m stop_on_loss=%s loss_stop_dist=%.2f m, visual_gains=[ext=%.2f pitch=%.2f max_ext=%.2f m/s max_pitch=%.1f deg/s period=%.2f s pitch_horizon=%.2f s step_ext=%.3f m step_pitch=%.1f deg slowdown_start=%.2f m near_scale=%.2f priority_err=%.3f m], camera_signs=[forward=%.1f vertical=%.1f]",
                        docking_standoff_m_, docking_extension_max_m_,
                        docking_pitch_limit_rad_ * 180.0 / M_PI,
                        docking_pitch_hold_cap_rad_ * 180.0 / M_PI,
                        docking_stage_extension_m_, docking_stage_pitch_rad_ * 180.0 / M_PI,
                        docking_stage_min_pitch_rad_ * 180.0 / M_PI,
                        docking_hold_on_lateral_error_ ? "true" : "false",
                        docking_lock_goal_camera_y_on_stage_complete_ ? "true" : "false",
                        docking_goal_camera_y_offset_m_,
                        docking_stop_on_close_tracking_loss_ ? "true" : "false",
                        docking_tracking_loss_stop_distance_m_,
                        docking_visual_extension_kp_,
                        docking_visual_pitch_kp_,
                        docking_visual_max_extension_rate_mps_,
                        docking_visual_max_pitch_rate_radps_ * 180.0 / M_PI,
                        docking_visual_command_period_sec_,
                        docking_visual_pitch_command_horizon_sec_,
                        docking_visual_extension_step_max_m_,
                        docking_visual_pitch_step_max_rad_ * 180.0 / M_PI,
                        docking_visual_slowdown_start_m_,
                        docking_visual_near_goal_scale_,
                        docking_visual_vertical_priority_error_m_,
                        docking_camera_forward_sign_, docking_camera_vertical_sign_);
                }
                break;

            case TurretState::TELEOP:
                // TELEOP mode allows direct velocity control without zeroing requirement
                // Enter MIT mode for pitch and yaw motors (if not already in teleop mode)
                if (current_state_ != TurretState::TELEOP_ZERO) {
                    if (turret_) {
                        turret_->EnterTeleopMode();
                    }
                }
                // Reset teleop velocities to zero when entering TELEOP state
                teleop_sz_velocity_ = 0.0;
                teleop_pitch_velocity_ = 0.0;
                teleop_yaw_velocity_ = 0.0;
                docking_enabled_ = false;
                docking_stage_active_ = false;
                current_state_ = TurretState::TELEOP;
                response->success = true;
                response->message = "State changed to TELEOP (direct motor control)";
                break;

            case TurretState::TELEOP_ZERO:
                // TELEOP_ZERO mode: teleop control while monitoring for zeroing events
                // Enter MIT mode for pitch and yaw motors (if not already in teleop mode)
                if (current_state_ != TurretState::TELEOP) {
                    if (turret_) {
                        turret_->EnterTeleopMode();
                    }
                }
                // Reset teleop velocities and zeroing flags
                teleop_sz_velocity_ = 0.0;
                teleop_pitch_velocity_ = 0.0;
                teleop_yaw_velocity_ = 0.0;
                sz_zeroed_ = false;
                pitch_zeroed_ = false;
                yaw_zeroed_ = false;
                is_zeroed_ = false;
                docking_enabled_ = false;
                docking_stage_active_ = false;
                current_state_ = TurretState::TELEOP_ZERO;
                response->success = true;
                response->message = "State changed to TELEOP_ZERO (zeroing mode)";
                RCLCPP_INFO(this->get_logger(),
                    "TELEOP_ZERO: Use joystick to retract SZ and pitch to limit switches. "
                    "Call zero_yaw service when yaw is at desired zero position.");
                break;

            default:
                response->success = false;
                response->message = "Invalid state requested";
                break;
        }
        
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "State transition: %s", response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "State transition failed: %s", response->message.c_str());
        }
    }

    void zeroYawCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // Unused

        // Only allow yaw zeroing in TELEOP_ZERO state
        if (current_state_ != TurretState::TELEOP_ZERO) {
            response->success = false;
            response->message = "Cannot zero yaw: must be in TELEOP_ZERO state";
            RCLCPP_WARN(this->get_logger(), "Zero yaw denied: not in TELEOP_ZERO state");
            return;
        }

        if (yaw_zeroed_) {
            response->success = false;
            response->message = "Yaw already zeroed";
            return;
        }

        try {
            if (turret_) {
                turret_->ZeroYawMotor();
                yaw_zeroed_ = true;
                response->success = true;
                response->message = "Yaw motor zeroed successfully";
                RCLCPP_INFO(this->get_logger(), "TELEOP_ZERO: Yaw motor zeroed");

                // Check if all components are now zeroed
                if (sz_zeroed_ && pitch_zeroed_ && yaw_zeroed_) {
                    is_zeroed_ = true;
                    RCLCPP_INFO(this->get_logger(), "TELEOP_ZERO: All components zeroed!");
                }
            } else {
                response->success = false;
                response->message = "Turret not initialized";
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to zero yaw: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Zero yaw failed: %s", e.what());
        }
    }

};

// LoadCellNode runs as a separate rclcpp::Node in the same process.
// Its implementation is in load_cell_ros2_node.cpp (linked into this executable).
#include "load_cell_node.h"

// Signal handler – just requests ROS shutdown; node destructors handle cleanup
void signalHandler(int /*signum*/)
{
    std::cout << "\nShutdown signal received – stopping..." << std::endl;
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Initialise pigpio once for the entire process.
    // Both TurretROS2Node and LoadCellNode share this single pigpio context.
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialise pigpio – aborting" << std::endl;
        return 1;
    }

    // Register signal handlers after pigpio (which installs its own handlers)
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // LoadCellNode is created FIRST so the HX711 chips initialise in a clean
        // GPIO environment – before the pi3hat (SPI1/GPIO 20-29) and encoder ISRs
        // (GPIO 16/19) are set up by TurretROS2Node.  Timer callbacks in both nodes
        // only start firing once executor.spin() is called, so ordering here only
        // affects GPIO initialisation, not runtime behaviour.
        auto load_cell_node = std::make_shared<LoadCellNode>();
        auto turret_node = std::make_shared<TurretROS2Node>();

        RCLCPP_INFO(turret_node->get_logger(),
            "Both nodes created – starting MultiThreadedExecutor");

        // MultiThreadedExecutor lets load cell and turret callbacks run in
        // parallel so slow HX711 bit-banging cannot block the control loop.
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(turret_node);
        executor.add_node(load_cell_node);
        executor.spin();

        // Destroy in reverse construction order
        turret_node.reset();
        load_cell_node.reset();

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
    }

    gpioTerminate();
    rclcpp::shutdown();
    std::cout << "Shutdown complete" << std::endl;
    return 0;
}

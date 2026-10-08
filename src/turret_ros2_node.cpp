#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "turret_control/msg/turret_state.hpp"
#include "turret_control/msg/zipper_command.hpp"
#include "turret_control/msg/turret_teleop_command.hpp"
#include "turret_control/msg/turret_velocities.hpp"
#include "turret_control/msg/load_cell_force.hpp"
#include "turret_control/srv/zero_turret.hpp"
#include "turret_control/srv/set_state.hpp"
#include "turret.h"
#include "autonomous_docking.h"
#include "docking/msg/docking_state.hpp"
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
        this->declare_parameter("teleop_yaw_brake_kp", 120.0);
        this->declare_parameter("teleop_yaw_brake_kd", 2.0);
        this->declare_parameter("undocking_pitch_only_duration_sec", 5.0);
        this->declare_parameter("undocking_combined_retract_duration_sec", 3.0);
        this->declare_parameter("undocking_target_extension_m", 0.2);
        this->declare_parameter("undocking_target_pitch_deg", 40.0);
        this->declare_parameter("undocking_retract_velocity", 1.5);
        
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
        declareDockingParameters();
        if (config_["undocking"]) {
            const auto undocking = config_["undocking"];
            if (undocking["pitch_only_duration_sec"]) {
                this->set_parameter(rclcpp::Parameter("undocking_pitch_only_duration_sec", undocking["pitch_only_duration_sec"].as<double>()));
            }
            if (undocking["combined_retract_duration_sec"]) {
                this->set_parameter(rclcpp::Parameter("undocking_combined_retract_duration_sec", undocking["combined_retract_duration_sec"].as<double>()));
            }
            if (undocking["target_extension_m"]) {
                this->set_parameter(rclcpp::Parameter("undocking_target_extension_m", undocking["target_extension_m"].as<double>()));
            }
            if (undocking["target_pitch_deg"]) {
                this->set_parameter(rclcpp::Parameter("undocking_target_pitch_deg", undocking["target_pitch_deg"].as<double>()));
            }
            if (undocking["retract_velocity"]) {
                this->set_parameter(rclcpp::Parameter("undocking_retract_velocity", undocking["retract_velocity"].as<double>()));
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
        teleop_yaw_brake_kp_ = this->get_parameter("teleop_yaw_brake_kp").as_double();
        teleop_yaw_brake_kd_ = this->get_parameter("teleop_yaw_brake_kd").as_double();
        
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
        yaw_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            topic_prefix_ + "/yaw_brake", 10,
            std::bind(&TurretROS2Node::yawBrakeCallback, this, std::placeholders::_1));

        // Subscribe to load cell force data published by LoadCellNode
        load_cell_force_sub_ = this->create_subscription<turret_control::msg::LoadCellForce>(
            topic_prefix_ + "/load_cell_force", 10,
            std::bind(&TurretROS2Node::loadCellForceCallback, this, std::placeholders::_1));
        vision_tag_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/end_effector_tag_pose_camera/" + this->get_parameter("docking_target_tag_frame").as_string(), 10,
            std::bind(&TurretROS2Node::visionTagPoseCallback, this, std::placeholders::_1));
        vision_tag_visible_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/vision/tag_visible/" + this->get_parameter("docking_target_tag_frame").as_string(), 10,
            std::bind(&TurretROS2Node::visionTagVisibleCallback, this, std::placeholders::_1));
        dock_state_sub_ = this->create_subscription<docking::msg::DockingState>(
            "/dock" + std::to_string(this->get_parameter("docking_target_dock_id").as_int()) + "/state", 10,
            std::bind(&TurretROS2Node::dockStateCallback, this, std::placeholders::_1));

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
        if (turret_) {
            turret_->RequestZeroAbort();
            turret_->StopAllMotors();
        }
        // pigpio is terminated in main() after all nodes are destroyed
    }

    void emergencyStop()
    {
        if (turret_) {
            turret_->RequestZeroAbort();
            turret_->StopAllMotors();
        }
        desired_velocity_ = 0.0;
        hold_current_pitch_ = true;
    }

private:
    enum class TurretState : uint8_t {
        IDLE = 0,
        READY = 1,
        RUNNING = 2,
        TELEOP = 3,
        TELEOP_ZERO = 4,
        DOCKING = 5,
        UNDOCKING = 6
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
    bool teleop_yaw_brake_requested_ = false;
    bool teleop_yaw_brake_active_ = false;
    double teleop_yaw_brake_target_rad_ = 0.0;
    double teleop_yaw_brake_kp_ = 120.0;
    double teleop_yaw_brake_kd_ = 2.0;
    using DockingController = turret_control::AutonomousDockingController;
    std::unique_ptr<DockingController> docking_controller_;
    DockingController::Output docking_output_{};
    bool have_docking_output_ = false;
    bool docking_actuation_failed_ = false;
    std::string docking_status_ = "Docking inactive";
    bool vision_tag_visible_ = false;
    bool have_tag_pose_ = false;
    double docking_pose_timeout_sec_ = 0.5;
    double docking_state_timeout_sec_ = 0.5;
    double docking_command_period_sec_ = 0.1;
    double docking_last_update_sec_ = 0.0;
    double docking_camera_forward_sign_ = 1.0;
    double docking_camera_vertical_sign_ = 1.0;
    std::string docking_camera_frame_ = "camera_link";
    // Also used by the existing undocking controller.
    double docking_extension_max_m_ = 0.90;
    double docking_pitch_limit_rad_ = 65.0 * M_PI / 180.0;
    double docking_pitch_hold_cap_rad_ = 63.0 * M_PI / 180.0;
    double undocking_pitch_only_duration_sec_ = 5.0;
    double undocking_combined_retract_duration_sec_ = 3.0;
    double undocking_target_extension_m_ = 0.2;
    double undocking_target_pitch_rad_ = 40.0 * M_PI / 180.0;
    double undocking_retract_velocity_ = 1.5;
    bool undocking_pitch_only_active_ = false;
    bool undocking_combined_active_ = false;
    bool undocking_target_reached_logged_ = false;
    rclcpp::Time undocking_pitch_only_deadline_{0, 0, RCL_ROS_TIME};
    rclcpp::Time undocking_combined_deadline_{0, 0, RCL_ROS_TIME};
    double undocking_combined_pitch_velocity_radps_ = 0.0;
    double yaw_hold_target_rad_ = 0.0;
    double yaw_hold_kp_ = 8.0;
    double yaw_hold_kd_ = 2.0;
    geometry_msgs::msg::PoseStamped latest_tag_pose_camera_;
    rclcpp::Time latest_tag_pose_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_tag_receipt_time_{0, 0, RCL_ROS_TIME};
    docking::msg::DockingState latest_dock_state_;
    bool have_dock_state_ = false;
    rclcpp::Time latest_dock_state_time_{0, 0, RCL_ROS_TIME};

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr yaw_brake_sub_;
    rclcpp::Subscription<turret_control::msg::LoadCellForce>::SharedPtr load_cell_force_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_tag_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_tag_visible_sub_;
    rclcpp::Subscription<docking::msg::DockingState>::SharedPtr dock_state_sub_;

    // Teleop command variables
    double teleop_sz_velocity_ = 0.0;
    double teleop_pitch_velocity_ = 0.0;
    double teleop_yaw_velocity_ = 0.0;

    void declareDockingParameters()
    {
        const auto docking = config_["docking"];
        auto declare_double = [&](const char *key, double fallback) {
            const double value = docking && docking[key] ? docking[key].as<double>() : fallback;
            this->declare_parameter(std::string("docking_") + key, value);
        };
        declare_double("goal_camera_x_m", 0.0);
        declare_double("goal_camera_y_m", 0.03);
        declare_double("insertion_distance_m", 0.10);
        declare_double("lateral_tolerance_m", 0.015);
        declare_double("vertical_tolerance_m", 0.02);
        declare_double("lateral_deadband_m", 0.005);
        declare_double("vertical_deadband_m", 0.005);
        declare_double("depth_deadband_m", 0.005);
        declare_double("pitch_min_deg", 0.0);
        declare_double("pitch_max_deg", 63.0);
        declare_double("yaw_limit_from_start_deg", 30.0);
        declare_double("pitch_down_sign", 1.0);
        declare_double("yaw_correction_sign", 1.0);
        declare_double("pitch_gain_rad_per_m", 10.0);
        declare_double("yaw_gain", 1.5);
        declare_double("extension_gain", 0.8);
        declare_double("max_pitch_rate_deg_s", 20.0);
        declare_double("max_yaw_rate_deg_s", 15.0);
        declare_double("max_extension_rate_m_s", 0.04);
        declare_double("max_pitch_step_deg", 2.0);
        declare_double("max_yaw_step_deg", 2.0);
        declare_double("max_extension_step_m", 0.01);
        declare_double("max_dt_sec", 0.1);
        declare_double("extension_min_m", 0.0);
        declare_double("extension_max_m", 0.90);
        declare_double("initial_search_extension_m", -1.0);
        declare_double("search_pitch_span_deg", 63.0);
        declare_double("search_pitch_rate_deg_s", 8.0);
        declare_double("search_timeout_sec", 20.0);
        declare_double("alignment_timeout_sec", 20.0);
        declare_double("approach_timeout_sec", 40.0);
        declare_double("zipper_motor_velocity_rad_s", 1.5);
        declare_double("recovery_retract_m", 0.03);
        declare_double("recovery_retract_velocity_rad_s", 0.5);
        declare_double("recovery_extension_tolerance_m", 0.003);
        declare_double("recovery_pitch_span_deg", 10.0);
        declare_double("recovery_pitch_endpoint_tolerance_deg", 1.0);
        declare_double("recovery_pitch_rate_deg_s", 8.0);
        declare_double("recovery_retract_timeout_sec", 8.0);
        declare_double("recovery_sweep_timeout_sec", 15.0);
        declare_double("insertion_velocity_rad_s", 0.3);
        declare_double("latch_timeout_sec", 8.0);
        declare_double("pose_timeout_sec", 0.5);
        declare_double("state_timeout_sec", 0.5);
        declare_double("command_period_sec", 0.1);
        declare_double("camera_forward_sign", 1.0);
        declare_double("camera_vertical_sign", 1.0);
        this->declare_parameter("docking_max_recovery_attempts",
            docking && docking["max_recovery_attempts"] ? docking["max_recovery_attempts"].as<int>() : 2);
        rcl_interfaces::msg::ParameterDescriptor target_descriptor;
        target_descriptor.read_only = true;
        target_descriptor.description = "Feedback subscription target; set at launch and restart to change";
        this->declare_parameter("docking_target_dock_id",
            docking && docking["target_dock_id"] ? docking["target_dock_id"].as<int>() : 1,
            target_descriptor);
        this->declare_parameter<std::string>("docking_camera_frame",
            docking && docking["camera_frame"] ? docking["camera_frame"].as<std::string>() : "camera_link");
        const int configured_turret_id = config_["turret_id"] ? config_["turret_id"].as<int>() : 1;
        this->declare_parameter<std::string>("docking_target_tag_frame",
            docking && docking["target_tag_frame"] ? docking["target_tag_frame"].as<std::string>() :
            "tag36h11_" + std::to_string(configured_turret_id - 1), target_descriptor);
        const auto tag_frame = this->get_parameter("docking_target_tag_frame").as_string();
        if (tag_frame.empty() || tag_frame.find('/') != std::string::npos ||
            tag_frame.find_first_of(" \t\r\n") != std::string::npos) {
            throw std::invalid_argument("docking_target_tag_frame must be a valid single topic component");
        }
        if (this->get_parameter("docking_target_dock_id").as_int() < 1) {
            throw std::invalid_argument("docking_target_dock_id must be positive");
        }
        loadDockingConfig();
    }

    DockingController::Config loadDockingConfig()
    {
        DockingController::Config settings;
        auto get = [&](const char *key) {
            return this->get_parameter(std::string("docking_") + key).as_double();
        };
        settings.goal_camera_x_m = get("goal_camera_x_m");
        settings.goal_camera_y_m = get("goal_camera_y_m");
        settings.insertion_distance_m = get("insertion_distance_m");
        settings.lateral_tolerance_m = get("lateral_tolerance_m");
        settings.vertical_tolerance_m = get("vertical_tolerance_m");
        settings.lateral_deadband_m = get("lateral_deadband_m");
        settings.vertical_deadband_m = get("vertical_deadband_m");
        settings.depth_deadband_m = get("depth_deadband_m");
        settings.pitch_min_rad = get("pitch_min_deg") * M_PI / 180.0;
        settings.pitch_max_rad = get("pitch_max_deg") * M_PI / 180.0;
        settings.yaw_limit_from_start_rad = get("yaw_limit_from_start_deg") * M_PI / 180.0;
        settings.pitch_down_sign = get("pitch_down_sign");
        settings.yaw_correction_sign = get("yaw_correction_sign");
        settings.pitch_gain_rad_per_m = get("pitch_gain_rad_per_m");
        settings.yaw_gain = get("yaw_gain");
        settings.extension_gain = get("extension_gain");
        settings.max_pitch_rate_rad_s = get("max_pitch_rate_deg_s") * M_PI / 180.0;
        settings.max_yaw_rate_rad_s = get("max_yaw_rate_deg_s") * M_PI / 180.0;
        settings.max_extension_rate_m_s = get("max_extension_rate_m_s");
        settings.max_pitch_step_rad = get("max_pitch_step_deg") * M_PI / 180.0;
        settings.max_yaw_step_rad = get("max_yaw_step_deg") * M_PI / 180.0;
        settings.max_extension_step_m = get("max_extension_step_m");
        settings.max_dt_sec = get("max_dt_sec");
        settings.extension_min_m = get("extension_min_m");
        settings.extension_max_m = get("extension_max_m");
        settings.initial_search_extension_m = get("initial_search_extension_m");
        settings.search_pitch_span_rad = get("search_pitch_span_deg") * M_PI / 180.0;
        settings.search_pitch_rate_rad_s = get("search_pitch_rate_deg_s") * M_PI / 180.0;
        settings.search_timeout_sec = get("search_timeout_sec");
        settings.alignment_timeout_sec = get("alignment_timeout_sec");
        settings.approach_timeout_sec = get("approach_timeout_sec");
        settings.zipper_motor_velocity_rad_s = get("zipper_motor_velocity_rad_s");
        settings.recovery_retract_m = get("recovery_retract_m");
        settings.recovery_retract_velocity_rad_s = get("recovery_retract_velocity_rad_s");
        settings.recovery_extension_tolerance_m = get("recovery_extension_tolerance_m");
        settings.recovery_pitch_span_rad = get("recovery_pitch_span_deg") * M_PI / 180.0;
        settings.recovery_pitch_endpoint_tolerance_rad = get("recovery_pitch_endpoint_tolerance_deg") * M_PI / 180.0;
        settings.recovery_pitch_rate_rad_s = get("recovery_pitch_rate_deg_s") * M_PI / 180.0;
        settings.recovery_retract_timeout_sec = get("recovery_retract_timeout_sec");
        settings.recovery_sweep_timeout_sec = get("recovery_sweep_timeout_sec");
        settings.insertion_velocity_rad_s = get("insertion_velocity_rad_s");
        settings.latch_timeout_sec = get("latch_timeout_sec");
        settings.max_recovery_attempts = static_cast<int>(this->get_parameter("docking_max_recovery_attempts").as_int());
        docking_pose_timeout_sec_ = get("pose_timeout_sec");
        docking_state_timeout_sec_ = get("state_timeout_sec");
        docking_command_period_sec_ = get("command_period_sec");
        docking_camera_forward_sign_ = get("camera_forward_sign");
        docking_camera_vertical_sign_ = get("camera_vertical_sign");
        docking_camera_frame_ = this->get_parameter("docking_camera_frame").as_string();
        if (!std::isfinite(docking_pose_timeout_sec_) || docking_pose_timeout_sec_ <= 0.0 ||
            !std::isfinite(docking_state_timeout_sec_) || docking_state_timeout_sec_ <= 0.0 ||
            !std::isfinite(docking_command_period_sec_) || docking_command_period_sec_ < 0.05 ||
            docking_command_period_sec_ > 0.5 || docking_camera_frame_.empty() ||
            std::fabs(docking_camera_forward_sign_) != 1.0 ||
            std::fabs(docking_camera_vertical_sign_) != 1.0) {
            throw std::invalid_argument("Invalid docking freshness, frame, signs or command period");
        }
        docking_extension_max_m_ = settings.extension_max_m;
        docking_pitch_limit_rad_ = std::max(std::fabs(settings.pitch_min_rad), std::fabs(settings.pitch_max_rad));
        docking_pitch_hold_cap_rad_ = settings.pitch_max_rad;
        return settings;
    }

    void releaseTeleopYawBrake()
    {
        teleop_yaw_brake_requested_ = false;
        teleop_yaw_brake_active_ = false;
    }

    void updateTeleopYawBrakeTarget()
    {
        if (!turret_ || !teleop_yaw_brake_requested_) {
            return;
        }

        if (!teleop_yaw_brake_active_) {
            teleop_yaw_brake_target_rad_ = turret_->GetYawAngle();
            teleop_yaw_brake_active_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Teleop yaw brake engaged at %.3f rad (%.1f deg) with kp=%.2f kd=%.2f",
                teleop_yaw_brake_target_rad_,
                teleop_yaw_brake_target_rad_ * 180.0 / M_PI,
                teleop_yaw_brake_kp_,
                teleop_yaw_brake_kd_);
        }
    }

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
            if (yaw_zeroed_ || current_state_ == TurretState::DOCKING) {
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

            case TurretState::UNDOCKING:
                if (is_zeroed_) {
                    executeZipperCommand();
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "In UNDOCKING state but not zeroed - cannot execute commands");
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
        try {
            if (!turret_) return;
            if (current_state_ == TurretState::DOCKING) {
                executeAutonomousDockingCommand();
                return;
            }
            if (current_state_ == TurretState::UNDOCKING) updateUndockingSetpoint();
            if (current_state_ == TurretState::UNDOCKING && undocking_combined_active_) {
                turret_->ActuateFixedVelocityPitchAndZipper(
                    static_cast<float>(desired_length_), static_cast<float>(std::abs(desired_velocity_)),
                    static_cast<float>(undocking_combined_pitch_velocity_radps_));
            } else {
                turret_->ActuateTurretCable(
                    static_cast<float>(desired_length_), static_cast<float>(desired_pitch_angle_),
                    static_cast<float>(std::abs(desired_velocity_)), hold_current_pitch_, false);
            }
            if (current_state_ == TurretState::UNDOCKING && yaw_zeroed_) {
                turret_->HoldYawPosition(yaw_hold_target_rad_, yaw_hold_kp_, yaw_hold_kd_);
            }
        } catch (const std::exception& e) {
            turret_->StopAllMotors();
            if (current_state_ == TurretState::DOCKING) {
                docking_actuation_failed_ = true;
                docking_status_ = std::string("FAILED: motor command error: ") + e.what();
            }
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
                if (teleop_yaw_brake_requested_) {
                    updateTeleopYawBrakeTarget();
                    turret_->HoldYawPosition(teleop_yaw_brake_target_rad_,
                                             teleop_yaw_brake_kp_,
                                             teleop_yaw_brake_kd_);
                } else {
                    teleop_yaw_brake_active_ = false;
                    turret_->SetYawVelocity(teleop_yaw_velocity_);
                }

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
        vel_msg.yaw_velocity = teleop_yaw_brake_requested_ ? 0.0 : teleop_yaw_velocity_;
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
                state_msg.status_message = docking_status_;
                break;
            case TurretState::UNDOCKING:
                state_msg.status_message = "Turret is UNDOCKING";
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
        const auto &p = msg->pose.position;
        const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
        const double age = (this->now() - stamp).seconds();
        if (msg->header.frame_id != docking_camera_frame_ || stamp.nanoseconds() == 0 ||
            age < -0.05 || age > docking_pose_timeout_sec_ ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0.0) {
            return;
        }
        if (have_tag_pose_ && stamp < latest_tag_pose_time_) return;
        latest_tag_pose_camera_ = *msg;
        latest_tag_pose_time_ = stamp; // Detection time; repeated cached TF cannot renew this.
        latest_tag_receipt_time_ = this->now();
        have_tag_pose_ = true;
    }

    void visionTagVisibleCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        vision_tag_visible_ = msg->data;
    }

    void dockStateCallback(const docking::msg::DockingState::SharedPtr msg)
    {
        latest_dock_state_ = *msg;
        latest_dock_state_time_ = this->now();
        have_dock_state_ = true;
    }

    DockingController::Input dockingInput() const
    {
        DockingController::Input input;
        input.now_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        input.extension_m = current_extension_;
        input.pitch_rad = current_pitch_angle_;
        input.yaw_rad = current_yaw_angle_;
        const auto now = this->now();
        const double pose_age = (now - latest_tag_pose_time_).seconds();
        const double receipt_age = (now - latest_tag_receipt_time_).seconds();
        input.pose_fresh = have_tag_pose_ && vision_tag_visible_ &&
            latest_tag_pose_camera_.header.frame_id == docking_camera_frame_ &&
            pose_age >= -0.05 && pose_age <= docking_pose_timeout_sec_ &&
            receipt_age >= 0.0 && receipt_age <= docking_pose_timeout_sec_;
        input.camera_x_m = latest_tag_pose_camera_.pose.position.x;
        input.camera_y_m = docking_camera_vertical_sign_ * latest_tag_pose_camera_.pose.position.y;
        input.camera_z_m = docking_camera_forward_sign_ * latest_tag_pose_camera_.pose.position.z;
        const double dock_age = (now - latest_dock_state_time_).seconds();
        input.dock_fresh = have_dock_state_ && dock_age >= 0.0 && dock_age <= docking_state_timeout_sec_;
        input.dock_detected = latest_dock_state_.dock_detected;
        input.dock_closing = latest_dock_state_.docking_state == docking::msg::DockingState::DOCKING;
        input.dock_complete = latest_dock_state_.docking_state == docking::msg::DockingState::DOCKED;
        return input;
    }

    void executeAutonomousDockingCommand()
    {
        if (!docking_controller_ || docking_actuation_failed_) {
            turret_->StopAllMotors();
            return;
        }
        const auto input = dockingInput();
        const auto phase = docking_controller_->phase();
        const bool tracking_lost = !input.pose_fresh &&
            (phase == DockingController::Phase::ALIGN || phase == DockingController::Phase::APPROACH);
        const bool contact = input.dock_fresh &&
            (input.dock_detected || input.dock_closing || input.dock_complete);
        const bool insertion_feedback_lost = !input.dock_fresh &&
            (phase == DockingController::Phase::INSERT || phase == DockingController::Phase::WAIT_FOR_LATCH);
        if (!have_docking_output_ || tracking_lost || contact || insertion_feedback_lost ||
            input.now_sec - docking_last_update_sec_ >= docking_command_period_sec_) {
            docking_output_ = docking_controller_->update(input);
            docking_last_update_sec_ = input.now_sec;
            have_docking_output_ = true;
            const auto status = docking_output_.phase == DockingController::Phase::FAILED
                ? std::string("Docking failed: ") + docking_output_.status : docking_output_.status;
            if (docking_status_ != status) {
                docking_status_ = status;
                RCLCPP_INFO(this->get_logger(), "%s", docking_status_.c_str());
            }
            // Publish real camera coordinates, without relabeling axes as base coordinates.
            if (input.pose_fresh) docking_camera_pub_->publish(latest_tag_pose_camera_);
            geometry_msgs::msg::PoseStamped goal;
            goal.header.stamp = this->now();
            goal.header.frame_id = docking_camera_frame_;
            goal.pose.position.x = this->get_parameter("docking_goal_camera_x_m").as_double();
            goal.pose.position.y = docking_camera_vertical_sign_ * this->get_parameter("docking_goal_camera_y_m").as_double();
            goal.pose.position.z = docking_camera_forward_sign_ * this->get_parameter("docking_insertion_distance_m").as_double();
            goal.pose.orientation.w = 1.0;
            docking_target_pub_->publish(goal);
        }
        // STOP bypasses both trajectory functions; their zero speed is not a stop command.
        if (docking_output_.actuation == DockingController::Actuation::STOP) {
            turret_->StopAllMotors();
            return;
        }
        if (docking_output_.actuation == DockingController::Actuation::INSERT_FREE_PITCH) {
            turret_->ActuateDockingInsertion(docking_output_.zipper_motor_velocity_rad_s);
        } else {
            turret_->ActuateTurretCable(
                static_cast<float>(docking_output_.extension_m),
                static_cast<float>(docking_output_.pitch_rad),
                static_cast<float>(docking_output_.zipper_motor_velocity_rad_s), false, true);
        }
        turret_->HoldYawPosition(docking_output_.yaw_rad, yaw_hold_kp_, yaw_hold_kd_);
    }

    void updateUndockingSetpoint()
    {
        if (!turret_ || !pitch_zeroed_) {
            return;
        }

        const double target_extension = std::clamp(
            undocking_target_extension_m_,
            0.0,
            docking_extension_max_m_);
        const double target_pitch = std::clamp(
            undocking_target_pitch_rad_,
            -docking_pitch_limit_rad_,
            docking_pitch_hold_cap_rad_);
        const bool pitch_only_phase_active =
            undocking_pitch_only_active_ &&
            this->now() < undocking_pitch_only_deadline_;

        desired_pitch_angle_ = target_pitch;
        hold_current_pitch_ = false;

        if (pitch_only_phase_active) {
            desired_length_ = current_extension_;
            desired_velocity_ = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Undocking pitch-only phase: ext=%.3f m target_pitch=%.2f deg current_pitch=%.2f deg time_left=%.2f s",
                current_extension_,
                target_pitch * 180.0 / M_PI,
                current_pitch_angle_ * 180.0 / M_PI,
                std::max(0.0, (undocking_pitch_only_deadline_ - this->now()).seconds()));
            return;
        }

        if (undocking_pitch_only_active_) {
            undocking_pitch_only_active_ = false;
            undocking_combined_active_ = undocking_combined_retract_duration_sec_ > 0.0;
            undocking_combined_deadline_ =
                this->now() + rclcpp::Duration::from_seconds(std::max(0.0, undocking_combined_retract_duration_sec_));
            const double retract_direction = (target_pitch < current_pitch_angle_) ? -1.0 : 1.0;
            undocking_combined_pitch_velocity_radps_ = retract_direction *
                std::max(0.2, std::fabs(undocking_retract_velocity_));
            RCLCPP_INFO(this->get_logger(),
                "Undocking combined retract phase armed: ext_target=%.3f m pitch_motor_vel=%.3f rad/s duration=%.2f s",
                target_extension,
                undocking_combined_pitch_velocity_radps_,
                undocking_combined_retract_duration_sec_);
        }

        if (undocking_combined_active_ &&
            this->now() < undocking_combined_deadline_ &&
            current_extension_ > (target_extension + 0.01)) {
            desired_length_ = target_extension;
            desired_velocity_ = std::abs(undocking_retract_velocity_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Undocking combined phase: cmd_ext=%.3f m current_ext=%.3f m pitch_motor_vel=%.3f rad/s time_left=%.2f s",
                desired_length_,
                current_extension_,
                undocking_combined_pitch_velocity_radps_,
                std::max(0.0, (undocking_combined_deadline_ - this->now()).seconds()));
            return;
        }

        undocking_combined_active_ = false;
        desired_length_ = target_extension;
        desired_velocity_ = std::abs(undocking_retract_velocity_);

        const bool extension_reached = current_extension_ <= (target_extension + 0.01);
        const bool pitch_reached =
            std::fabs(current_pitch_angle_ - target_pitch) <= (2.0 * M_PI / 180.0);
        if (extension_reached && pitch_reached) {
            desired_velocity_ = 0.0;
            if (!undocking_target_reached_logged_) {
                undocking_target_reached_logged_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Undocking target reached: ext=%.3f m pitch=%.2f deg. Holding pose.",
                    current_extension_,
                    current_pitch_angle_ * 180.0 / M_PI);
            }
        } else {
            undocking_target_reached_logged_ = false;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Undocking retract phase: cmd_ext=%.3f m hold_pitch=%.2f deg current_ext=%.3f m current_pitch=%.2f deg vel=%.2f",
                desired_length_,
                desired_pitch_angle_ * 180.0 / M_PI,
                current_extension_,
                current_pitch_angle_ * 180.0 / M_PI,
                desired_velocity_);
        }
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
            turret_->RequestZeroAbort();
            turret_->StopAllMotors();
            RCLCPP_INFO(this->get_logger(), "All turret motors stopped");
        }
    }

    void resetDockingState()
    {
        docking_controller_.reset();
        have_docking_output_ = false;
        docking_actuation_failed_ = false;
        docking_last_update_sec_ = 0.0;
        docking_status_ = "Docking inactive";
    }

    void resetUndockingState()
    {
        undocking_pitch_only_active_ = false;
        undocking_combined_active_ = false;
        undocking_target_reached_logged_ = false;
        undocking_pitch_only_deadline_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        undocking_combined_deadline_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        undocking_combined_pitch_velocity_radps_ = 0.0;
    }
    
    void ensureMotorStopped()
    {
        if (!turret_) {
            return;
        }
        const bool zipper_moving = std::abs(current_velocity_) > 0.01;
        const bool pitch_moving = std::abs(turret_->GetPitchMotorVelocity()) > 0.01;
        const bool yaw_moving = std::abs(turret_->GetYawMotorVelocity()) > 0.01;
        if (zipper_moving || pitch_moving || yaw_moving) {
            turret_->StopAllMotors();
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
        // Keep teleop logging out of INFO; this callback can run at high rate.
        if (std::abs(teleop_sz_velocity_) > 0.001 ||
            std::abs(teleop_pitch_velocity_) > 0.001 ||
            std::abs(teleop_yaw_velocity_) > 0.001) {
            RCLCPP_DEBUG(this->get_logger(),
                "RX Teleop CMD: sz=%.3f, pitch=%.3f, yaw=%.3f",
                teleop_sz_velocity_, teleop_pitch_velocity_, teleop_yaw_velocity_);
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                "Teleop command: sz_vel=%.3f, pitch_vel=%.3f, yaw_vel=%.3f",
                teleop_sz_velocity_, teleop_pitch_velocity_, teleop_yaw_velocity_);
        }
    }

    void yawBrakeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (current_state_ != TurretState::TELEOP) {
            if (msg->data) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Ignoring yaw brake request outside TELEOP");
            }
            releaseTeleopYawBrake();
            return;
        }

        const bool previous_requested = teleop_yaw_brake_requested_;
        teleop_yaw_brake_requested_ = msg->data;
        if (!teleop_yaw_brake_requested_) {
            teleop_yaw_brake_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Teleop yaw brake released");
        } else if (!previous_requested) {
            teleop_yaw_brake_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Teleop yaw brake requested");
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
            turret_->ClearZeroAbort();
            turret_->StopAllMotors();
            
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
                resetDockingState();
                resetUndockingState();
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
                    resetDockingState();
                    resetUndockingState();
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
                    resetDockingState();
                    resetUndockingState();
                    current_state_ = TurretState::RUNNING;
                    response->success = true;
                    response->message = "State changed to RUNNING";
                }
                break;

            case TurretState::DOCKING:
                if (is_zeroing_) {
                    response->success = false;
                    response->message = "Cannot go to DOCKING: zeroing is still in progress";
                } else if (!is_zeroed_) {
                    response->success = false;
                    response->message = "Cannot go to DOCKING: turret not zeroed";
                } else {
                    try {
                        auto settings = loadDockingConfig();
                        resetDockingState();
                        resetUndockingState();
                        releaseTeleopYawBrake();
                        turret_->EnableControlMotors();
                        // Use the existing zero -> READY prerequisite. Yaw corrections
                        // are relative to actual motor feedback, without a new zeroing gate.
                        turret_->SetYawVelocity(0.0);
                        current_yaw_angle_ = turret_->GetYawAngle();
                        docking_controller_ = std::make_unique<DockingController>(settings);
                        const auto initial_input = dockingInput();
                        if (!initial_input.dock_fresh) {
                            throw std::runtime_error("No fresh state from the configured docking port");
                        }
                        docking_controller_->start(initial_input);
                        if (docking_controller_->phase() == DockingController::Phase::FAILED) {
                            throw std::runtime_error(docking_controller_->update(initial_input).status);
                        }
                        current_state_ = TurretState::DOCKING;
                        response->success = true;
                        response->message = "Docking started: pitch search, align, approach, insert until contact";
                        executeAutonomousDockingCommand();
                    } catch (const std::exception& e) {
                        turret_->StopAllMotors();
                        resetDockingState();
                        current_state_ = TurretState::READY;
                        response->success = false;
                        response->message = std::string("Cannot start docking: ") + e.what();
                    }
                }
                break;

            case TurretState::UNDOCKING:
                if (!is_zeroed_) {
                    response->success = false;
                    response->message = "Cannot go to UNDOCKING: turret not zeroed";
                } else {
                    resetDockingState();
                    resetUndockingState();
                    undocking_pitch_only_duration_sec_ =
                        this->get_parameter("undocking_pitch_only_duration_sec").as_double();
                    undocking_combined_retract_duration_sec_ =
                        this->get_parameter("undocking_combined_retract_duration_sec").as_double();
                    undocking_target_extension_m_ =
                        this->get_parameter("undocking_target_extension_m").as_double();
                    undocking_target_pitch_rad_ =
                        this->get_parameter("undocking_target_pitch_deg").as_double() * M_PI / 180.0;
                    undocking_retract_velocity_ =
                        this->get_parameter("undocking_retract_velocity").as_double();
                    undocking_pitch_only_active_ = undocking_pitch_only_duration_sec_ > 0.0;
                    undocking_pitch_only_deadline_ =
                        this->now() + rclcpp::Duration::from_seconds(std::max(0.0, undocking_pitch_only_duration_sec_));
                    desired_length_ = current_extension_;
                    desired_velocity_ = 0.0;
                    desired_pitch_angle_ = undocking_target_pitch_rad_;
                    hold_current_pitch_ = false;
                    yaw_hold_target_rad_ = current_yaw_angle_;
                    current_state_ = TurretState::UNDOCKING;
                    response->success = true;
                    response->message = "State changed to UNDOCKING";
                    RCLCPP_INFO(this->get_logger(),
                        "Undocking armed: pitch_only=%.2f s combined=%.2f s target_ext=%.3f m target_pitch=%.1f deg retract_vel=%.2f",
                        undocking_pitch_only_duration_sec_,
                        undocking_combined_retract_duration_sec_,
                        undocking_target_extension_m_,
                        undocking_target_pitch_rad_ * 180.0 / M_PI,
                        undocking_retract_velocity_);
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
                resetDockingState();
                resetUndockingState();
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
                resetDockingState();
                resetUndockingState();
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
        
        response->current_state = static_cast<uint8_t>(current_state_);
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

        // Allow yaw zeroing in IDLE or TELEOP_ZERO state
        if (current_state_ != TurretState::TELEOP_ZERO &&
            current_state_ != TurretState::IDLE) {
            response->success = false;
            response->message = "Cannot zero yaw: must be in IDLE or TELEOP_ZERO state";
            RCLCPP_WARN(this->get_logger(), "Zero yaw denied: not in IDLE or TELEOP_ZERO state");
            return;
        }

        if (yaw_zeroed_ && current_state_ == TurretState::TELEOP_ZERO) {
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
static std::weak_ptr<TurretROS2Node> g_turret_node;

void signalHandler(int /*signum*/)
{
    std::cout << "\nShutdown signal received – stopping..." << std::endl;
    if (auto turret_node = g_turret_node.lock()) {
        turret_node->emergencyStop();
    }
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
        // GPIO environment - before the pi3hat (SPI1/GPIO 20-29) and encoder ISRs
        // (GPIO 16/19) are set up by TurretROS2Node. Timer callbacks in both nodes
        // only start firing once executor.spin() is called, so ordering here only
        // affects GPIO initialisation, not runtime behaviour.
        auto load_cell_node = std::make_shared<LoadCellNode>();
        auto turret_node = std::make_shared<TurretROS2Node>();
        g_turret_node = turret_node;

        RCLCPP_INFO(turret_node->get_logger(),
            "Both nodes created - starting MultiThreadedExecutor");

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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "turret_control/msg/turret_state.hpp"
#include "turret_control/msg/zipper_command.hpp"
#include "turret_control/msg/turret_teleop_command.hpp"
#include "turret_control/msg/turret_velocities.hpp"
#include "turret_control/msg/load_cell_force.hpp"
#include "turret_control/srv/zero_turret.hpp"
#include "turret_control/srv/set_state.hpp"
#include "turret.h"
#include "config.h"
#include "load_cell.h"
#include <chrono>
#include <memory>
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
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
        // Initialize pigpio
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio");
            rclcpp::shutdown();
            return;
        }

        // Declare parameter for config path
        this->declare_parameter("config_path", "config/config.yaml");
        
        // Declare zero velocity parameter
        this->declare_parameter("zero_velocity", -0.5);
        
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
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize turret: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Initialize state machine FIRST - we'll be in IDLE during load cell init
        current_state_ = TurretState::IDLE;
        is_zeroed_ = false;
        is_zeroing_ = false;
        current_extension_ = 0.0;
        current_velocity_ = 0.0;
        desired_length_ = 0.0;  // Initialize to prevent oscillation
        desired_velocity_ = 0.0;  // Initialize to prevent oscillation
        zero_velocity_ = -0.3;  // Match the original zero function default
        
        RCLCPP_INFO(this->get_logger(), "State machine initialized - State: IDLE, Zeroed: %s", 
            is_zeroed_ ? "true" : "false");
        
        // Publishers
        heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>(topic_prefix_ + "/heartbeat", 10);
        state_pub_ = this->create_publisher<turret_control::msg::TurretState>(topic_prefix_ + "/state", 10);
        velocities_pub_ = this->create_publisher<turret_control::msg::TurretVelocities>(
            "cmd_vel/" + topic_prefix_ + "/velocities", 10);
        load_cell_pub_ = this->create_publisher<turret_control::msg::LoadCellForce>(
            topic_prefix_ + "/load_cell_force", 10);

        // Subscribers
        zipper_cmd_sub_ = this->create_subscription<turret_control::msg::ZipperCommand>(
            topic_prefix_ + "/zipper_command", 10,
            std::bind(&TurretROS2Node::zipperCommandCallback, this, std::placeholders::_1));

        teleop_cmd_sub_ = this->create_subscription<turret_control::msg::TurretTeleopCommand>(
            topic_prefix_ + "/teleop_command", 10,
            std::bind(&TurretROS2Node::teleopCommandCallback, this, std::placeholders::_1));

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

        // Initialize load cells in MAIN THREAD (while state machine is in IDLE)
        // This must happen in the main thread due to pigpio/GPIO requirements
        RCLCPP_INFO(this->get_logger(), "Initializing load cells in main thread (state: IDLE)...");
        RCLCPP_INFO(this->get_logger(), "Waiting 10 seconds for HX711 chips to stabilize...");
        std::this_thread::sleep_for(std::chrono::seconds(10));

        initializeLoadCells();

        // Start load cell reading/publishing thread (only if initialization succeeded)
        if (load_cell_ready_ && load_cell_) {
            startLoadCellThread();
        } else {
            RCLCPP_WARN(this->get_logger(), "Load cells not initialized - thread will not start");
        }
    }

    ~TurretROS2Node()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down turret node - cleaning up resources...");

        // Step 1: Stop load cell thread first
        if (load_cell_thread_running_) {
            RCLCPP_INFO(this->get_logger(), "Stopping load cell thread...");
            load_cell_thread_running_ = false;
            if (load_cell_thread_.joinable()) {
                load_cell_thread_.join();
            }
            RCLCPP_INFO(this->get_logger(), "Load cell thread stopped");
        }

        // Step 2: Explicitly destroy load cell object (powers down HX711 chips)
        if (load_cell_) {
            RCLCPP_INFO(this->get_logger(), "Destroying load cell object (powering down HX711 chips)...");
            load_cell_.reset();  // Calls LoadCell destructor which powers down chips
            RCLCPP_INFO(this->get_logger(), "Load cell hardware powered down");
        }

        // Step 3: Terminate GPIO last
        RCLCPP_INFO(this->get_logger(), "Terminating pigpio...");
        gpioTerminate();
        RCLCPP_INFO(this->get_logger(), "Turret node shutdown complete");
    }

private:
    enum class TurretState : uint8_t {
        IDLE = 0,
        READY = 1,
        RUNNING = 2,
        TELEOP = 3,
        TELEOP_ZERO = 4
    };

    // Core components
    std::unique_ptr<Turret> turret_;
    std::unique_ptr<LoadCell> load_cell_;
    YAML::Node config_;
    int turret_id_;
    std::string topic_prefix_;
    bool load_cell_ready_;
    
    // State machine variables
    TurretState current_state_;
    bool is_zeroed_;
    bool is_zeroing_;
    double current_extension_;
    double current_velocity_;
    double desired_length_;
    double desired_velocity_;
    double zero_velocity_;
    std::future<bool> zero_future_;

    // Teleop zero state tracking
    bool sz_zeroed_ = false;       // Spiral zipper zeroed
    bool pitch_zeroed_ = false;    // Pitch encoder zeroed
    bool yaw_zeroed_ = false;      // Yaw motor zeroed
    double current_pitch_angle_ = 0.0;  // Current pitch angle from encoder
    double current_yaw_angle_ = 0.0;    // Current yaw angle from motor feedback
    
    
    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
    rclcpp::Publisher<turret_control::msg::TurretState>::SharedPtr state_pub_;
    rclcpp::Publisher<turret_control::msg::TurretVelocities>::SharedPtr velocities_pub_;
    rclcpp::Publisher<turret_control::msg::LoadCellForce>::SharedPtr load_cell_pub_;

    // ROS2 subscribers
    rclcpp::Subscription<turret_control::msg::ZipperCommand>::SharedPtr zipper_cmd_sub_;
    rclcpp::Subscription<turret_control::msg::TurretTeleopCommand>::SharedPtr teleop_cmd_sub_;

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

    // Load cell thread members
    std::thread load_cell_thread_;
    std::atomic<bool> load_cell_thread_running_{false};
    std::mutex load_cell_mutex_;
    double latest_force_{0.0};
    bool latest_force_valid_{false};

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
                    current_state_ = TurretState::READY;
                    current_extension_ = 0.0;
                    current_velocity_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Turret zeroing completed successfully! State changed to READY");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Turret zeroing failed! Returning to IDLE state");
                    current_state_ = TurretState::IDLE;
                }
            } else {
                // Still zeroing - log progress occasionally
                static int counter = 0;
                if (++counter % 100 == 0) {  // Log every ~1 second (at 100Hz update rate)
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
                // Convert velocity from m/s to rad/s
                // The spiral zipper mechanics use: extension = encoder_count * (extension_per_step * 4)
                // From config: extension_per_step = 0.000004453125, so meters_per_count = 0.000017812500
                // For a spiral mechanism, we need to convert linear velocity to angular velocity
                // For now, we'll use the velocity directly as it appears to be in correct units already
                double max_velocity = std::abs(desired_velocity_);

                if (max_velocity > 0.0) {
                    turret_->ActuateSpiralZipperLength(desired_length_, max_velocity);
                } else {
                    // Fallback to position-only control if no velocity specified
                    turret_->ActuateSpiralZipperLength(desired_length_);
                }

                RCLCPP_INFO(this->get_logger(),
                    "EXECUTING zipper command: length=%.3f meters, max_velocity=%.3f",
                    desired_length_, max_velocity);
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
                // Log motor commands when non-zero (throttled to avoid spam)
                if (std::abs(teleop_sz_velocity_) > 0.001 || std::abs(teleop_pitch_velocity_) > 0.001 || std::abs(teleop_yaw_velocity_) > 0.001) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "EXEC Teleop: Commanding motors sz=%.3f, pitch=%.3f, yaw=%.3f",
                        teleop_sz_velocity_, teleop_pitch_velocity_, teleop_yaw_velocity_);
                }

                turret_->SetSpiralZipperVelocity(teleop_sz_velocity_);
                turret_->SetPitchVelocity(teleop_pitch_velocity_);
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

            // Execute velocity commands same as teleop
            turret_->SetSpiralZipperVelocity(teleop_sz_velocity_);
            turret_->SetPitchVelocity(teleop_pitch_velocity_);
            turret_->SetYawVelocity(teleop_yaw_velocity_);

            // Publish velocity commands
            publishVelocities();

            // Monitor spiral zipper limit switch for zeroing
            // The SZ zeroing is detected when extension is near zero after retracting
            // For now, we check if extension is very small (near limit)
            if (!sz_zeroed_ && current_extension_ < 0.001 && teleop_sz_velocity_ < 0) {
                // SZ has been retracted to limit - zero the encoder
                turret_->ZeroSpiralZipper();  // This will reset the encoder
                sz_zeroed_ = true;
                RCLCPP_INFO(this->get_logger(), "TELEOP_ZERO: Spiral zipper zeroed");
            }

            // Monitor pitch limit switch for zeroing
            if (!pitch_zeroed_ && turret_->IsPitchLimitPressed()) {
                // Pitch limit switch pressed - zero the pitch encoder
                turret_->ZeroPitchEncoder();
                pitch_zeroed_ = true;
                RCLCPP_INFO(this->get_logger(), "TELEOP_ZERO: Pitch encoder zeroed");
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
        state_msg.load_cell_ready = load_cell_ready_;

        // Get force data from load cell thread (non-blocking, thread-safe)
        {
            std::lock_guard<std::mutex> lock(load_cell_mutex_);
            if (latest_force_valid_) {
                state_msg.force = latest_force_;
            } else {
                state_msg.force = 0.0;
            }
        }

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
    
    void initializeLoadCells()
    {
        load_cell_ready_ = false;

        try {
            RCLCPP_INFO(this->get_logger(), "Creating LoadCell object (initializing HX711 hardware)...");
            load_cell_ = std::make_unique<LoadCell>();
            RCLCPP_INFO(this->get_logger(), "LoadCell hardware initialized successfully!");

            // Try different paths for load cell config file, similar to main config
            std::string config_filename = "turret_" + std::to_string(turret_id_) + "_lc_config.cfg";
            std::vector<std::string> config_paths = {
                "config/" + config_filename,
                "../config/" + config_filename,
                "/home/turret/ros_ws/src/turret_control/config/" + config_filename,
                config_filename  // Try current directory as fallback
            };

            bool loaded = false;
            RCLCPP_INFO(this->get_logger(), "Attempting to load calibration file: %s", config_filename.c_str());

            for (const auto& config_path : config_paths) {
                RCLCPP_INFO(this->get_logger(), "Trying calibration path: %s", config_path.c_str());
                if (load_cell_->loadCalibrationData(config_path)) {
                    load_cell_ready_ = true;
                    loaded = true;
                    RCLCPP_INFO(this->get_logger(), "Load cells initialized successfully with config: %s", config_path.c_str());
                    break;
                }
            }

            if (!loaded) {
                RCLCPP_WARN(this->get_logger(), "Failed to load load cell calibration from any attempted path:");
                for (const auto& path : config_paths) {
                    RCLCPP_WARN(this->get_logger(), "  - %s", path.c_str());
                }
                RCLCPP_WARN(this->get_logger(), "Load cells will not be available until calibrated");
                RCLCPP_INFO(this->get_logger(), "To calibrate: ros2 run turret_control calibrate_load_cells config/%s", config_filename.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize load cells: %s", e.what());
            load_cell_ready_ = false;
        }
    }

    void startLoadCellThread()
    {
        // Load cells are already initialized in main thread
        // This thread just reads and publishes
        load_cell_thread_running_ = true;
        load_cell_thread_ = std::thread(&TurretROS2Node::loadCellThreadFunction, this);
        RCLCPP_INFO(this->get_logger(), "Load cell reading/publishing thread started");
    }

    void loadCellThreadFunction()
    {
        // Load cells are already initialized in the main thread
        // This thread ONLY reads and publishes data
        RCLCPP_INFO(this->get_logger(), "Load cell thread started - beginning data acquisition");

        // Publishing rate for load cell data (configurable, default 50Hz)
        const auto publish_rate = std::chrono::milliseconds(20);  // 50Hz

        while (load_cell_thread_running_ && rclcpp::ok()) {
            auto start_time = std::chrono::steady_clock::now();

            try {
                if (load_cell_ && load_cell_ready_) {
                    // Read force from load cells
                    double force = load_cell_->getForce("N");

                    // Update latest force with mutex protection
                    {
                        std::lock_guard<std::mutex> lock(load_cell_mutex_);
                        latest_force_ = force;
                        latest_force_valid_ = true;
                    }

                    // Publish load cell force message
                    auto msg = turret_control::msg::LoadCellForce();
                    msg.header.stamp = this->now();
                    msg.header.frame_id = topic_prefix_;
                    msg.force = force;
                    msg.calibrated = load_cell_ready_;

                    load_cell_pub_->publish(msg);

                    // Log occasionally for debugging (every 2 seconds at 50Hz = 100 iterations)
                    static int log_counter = 0;
                    if (++log_counter >= 100) {
                        RCLCPP_DEBUG(this->get_logger(), "Load cell force: %.2f N", force);
                        log_counter = 0;
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Error reading load cell in thread: %s", e.what());

                // Mark force as invalid on error
                {
                    std::lock_guard<std::mutex> lock(load_cell_mutex_);
                    latest_force_valid_ = false;
                }
            }

            // Sleep to maintain publish rate
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            auto sleep_time = publish_rate - elapsed;

            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Load cell thread exiting");
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
            "ZIPPER COMMAND RECEIVED: length=%.3f, velocity=%.3f",
            msg->desired_length, msg->desired_velocity);

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

        RCLCPP_INFO(this->get_logger(),
            "Zipper command accepted: length=%.3f, velocity=%.3f",
            desired_length_, desired_velocity_);
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
        
        // Validate velocity range for safety
        if (requested_velocity > 0 || requested_velocity < -3.0) {
            response->success = false;
            response->message = "Invalid velocity: must be negative and >= -3.0 rad/s for safety";
            RCLCPP_WARN(this->get_logger(), "Zero request denied: unsafe velocity %.2f", requested_velocity);
            return;
        }
        
        try {
            RCLCPP_INFO(this->get_logger(), "Starting turret zero sequence...");
            
            // Start zeroing in a separate thread to avoid blocking the executor
            is_zeroing_ = true;
            zero_velocity_ = requested_velocity;  // Use requested velocity
            
            zero_future_ = std::async(std::launch::async, [this]() -> bool {
                try {
                    turret_->ZeroSpiralZipper(zero_velocity_);
                    return true;
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
                    current_state_ = TurretState::RUNNING;
                    response->success = true;
                    response->message = "State changed to RUNNING";
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

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Initiating graceful shutdown..." << std::endl;
    rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
    // Register signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    rclcpp::init(argc, argv);

    std::shared_ptr<TurretROS2Node> node;

    try {
        node = std::make_shared<TurretROS2Node>();

        RCLCPP_INFO(node->get_logger(), "Turret control node created successfully");
        RCLCPP_INFO(node->get_logger(), "Starting to spin - ready to process callbacks...");

        // Try SingleThreadedExecutor first to isolate threading issues
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        RCLCPP_INFO(node->get_logger(), "Executor created, starting to spin...");

        // Spin with timeout to check if executor is working
        while (rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(100));
            // This should allow us to see if callbacks are being processed
        }

        RCLCPP_INFO(node->get_logger(), "Exiting spin loop - cleaning up...");

    } catch (const std::exception& e) {
        std::cerr << "Node initialization or runtime error: " << e.what() << std::endl;
    }

    // Explicitly destroy node before shutting down rclcpp
    // This ensures destructor is called and resources are cleaned up properly
    if (node) {
        std::cout << "Destroying node object..." << std::endl;
        node.reset();  // Calls ~TurretROS2Node() destructor
        std::cout << "Node destroyed" << std::endl;
    }

    rclcpp::shutdown();
    std::cout << "ROS2 shutdown complete" << std::endl;
    return 0;
}
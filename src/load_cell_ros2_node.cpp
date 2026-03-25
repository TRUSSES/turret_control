#include "load_cell_node.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <pigpio.h>
#include <signal.h>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// LoadCellNode implementation
// ---------------------------------------------------------------------------

LoadCellNode::LoadCellNode() : Node("load_cell_node")
{
    int turret_id = readTurretId();
    topic_prefix_ = "turret" + std::to_string(turret_id);

    RCLCPP_INFO(this->get_logger(),
        "Load cell node starting for topic prefix: %s", topic_prefix_.c_str());

    load_cell_pub_ = this->create_publisher<turret_control::msg::LoadCellForce>(
        topic_prefix_ + "/load_cell_force", 10);

    tare_service_ = this->create_service<std_srvs::srv::Trigger>(
        topic_prefix_ + "/tare_load_cells",
        std::bind(&LoadCellNode::tareCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Hardware init happens in the constructor (main thread, before spin() starts)
    // so pigpio bit-banging is set up before any timer callbacks fire.
    initializeLoadCells(turret_id);

    if (load_cell_ready_) {
        // Each getForce() call blocks for ~100 ms per active chip (HX711 at 10 SPS,
        // 1 sample per chip).  Setting the timer period longer than the callback
        // duration avoids the executor accumulating pending work items.
        // 3 chips × 1 sample = ~300 ms -> timer period = 350 ms ~= ~3 Hz.
        publish_timer_ = this->create_wall_timer(
            350ms, std::bind(&LoadCellNode::publishCallback, this));
        RCLCPP_INFO(this->get_logger(), "Load cell publish timer started (~3 Hz)");
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Load cells not ready – publishing will not start. "
            "Run calibrate_load_cells to generate a calibration file then restart.");
    }
}

LoadCellNode::~LoadCellNode()
{
    if (publish_timer_) {
        publish_timer_->cancel();
    }
    if (load_cell_) {
        load_cell_.reset();  // calls HX711::power_down() in LoadCell destructor
        RCLCPP_INFO(this->get_logger(), "Load cell hardware powered down");
    }
}

int LoadCellNode::readTurretId()
{
    std::vector<std::string> paths = {
        "config/config.yaml",
        "../config/config.yaml",
        "/home/turret/ros_ws/src/turret_control/config/config.yaml"
    };
    for (const auto& p : paths) {
        try {
            YAML::Node cfg = YAML::LoadFile(p);
            if (cfg["turret_id"]) {
                return cfg["turret_id"].as<int>();
            }
        } catch (...) {}
    }
    RCLCPP_WARN(this->get_logger(),
        "Could not read turret_id from config, defaulting to 1");
    return 1;
}

void LoadCellNode::initializeLoadCells(int turret_id)
{
    load_cell_ready_ = false;
    try {
        RCLCPP_INFO(this->get_logger(), "Resetting HX711 GPIO pins...");
        LoadCell::ResetGPIOPins();

        // 10 s wait empirically required: the HX711 chips need their full power-up
        // calibration (~400 ms) plus several conversion cycles, and Linux scheduling
        // jitter means a shorter wait can race with a mid-conversion DOUT HIGH state.
        RCLCPP_INFO(this->get_logger(), "Waiting 10 s for HX711 chips to stabilize...");
        rclcpp::sleep_for(10s);

        RCLCPP_INFO(this->get_logger(), "Creating LoadCell object...");
        load_cell_ = std::make_unique<LoadCell>();
        RCLCPP_INFO(this->get_logger(), "LoadCell hardware initialized");

        std::string cfg_file = "turret_" + std::to_string(turret_id) + "_lc_config.cfg";
        std::vector<std::string> cfg_paths = {
            "config/" + cfg_file,
            "../config/" + cfg_file,
            "/home/turret/ros_ws/src/turret_control/config/" + cfg_file,
            cfg_file
        };

        for (const auto& path : cfg_paths) {
            RCLCPP_INFO(this->get_logger(), "Trying calibration: %s", path.c_str());
            if (load_cell_->loadCalibrationData(path)) {
                load_cell_ready_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Calibration loaded from: %s", path.c_str());
                break;
            }
        }

        if (!load_cell_ready_) {
            RCLCPP_WARN(this->get_logger(),
                "No calibration file found. Run: "
                "ros2 run turret_control calibrate_load_cells config/%s",
                cfg_file.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
            "First init attempt failed (%s) – power-cycling chips and retrying...", e.what());
        load_cell_.reset();  // destroy partial object if any

        try {
            LoadCell::ResetGPIOPins();
            RCLCPP_INFO(this->get_logger(), "Retry: waiting 10 s for chips to stabilize...");
            rclcpp::sleep_for(10s);

            load_cell_ = std::make_unique<LoadCell>();
            RCLCPP_INFO(this->get_logger(), "LoadCell hardware initialized on retry");

            std::string cfg_file = "turret_" + std::to_string(turret_id) + "_lc_config.cfg";
            std::vector<std::string> cfg_paths = {
                "config/" + cfg_file,
                "../config/" + cfg_file,
                "/home/turret/ros_ws/src/turret_control/config/" + cfg_file,
                cfg_file
            };
            for (const auto& path : cfg_paths) {
                if (load_cell_->loadCalibrationData(path)) {
                    load_cell_ready_ = true;
                    RCLCPP_INFO(this->get_logger(), "Calibration loaded from: %s", path.c_str());
                    break;
                }
            }
            if (!load_cell_ready_) {
                RCLCPP_WARN(this->get_logger(), "Retry succeeded but no calibration file found.");
            }
        } catch (const std::exception& e2) {
            RCLCPP_ERROR(this->get_logger(),
                "Retry also failed: %s. Load cells unavailable.", e2.what());
            load_cell_.reset();
            load_cell_ready_ = false;
        }
    }
}

void LoadCellNode::publishCallback()
{
    if (!load_cell_ || !load_cell_ready_) return;

    try {
        double force = load_cell_->getForce("N");

        auto msg = turret_control::msg::LoadCellForce();
        msg.header.stamp = this->now();
        msg.header.frame_id = topic_prefix_;
        msg.force = force;
        msg.calibrated = true;
        load_cell_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Error reading load cell: %s", e.what());
    }
}

void LoadCellNode::tareCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!load_cell_) {
        response->success = false;
        response->message = "Load cells not initialized";
        return;
    }
    if (publish_timer_) publish_timer_->cancel();
    try {
        load_cell_->tare();
        response->success = true;
        response->message = "Load cells tared successfully";
        RCLCPP_INFO(this->get_logger(), "Load cells tared");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Tare failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "Tare failed: %s", e.what());
    }
    if (load_cell_ready_) {
        publish_timer_ = this->create_wall_timer(
            20ms, std::bind(&LoadCellNode::publishCallback, this));
    }
}

// ---------------------------------------------------------------------------
// Standalone main – used only when building the load_cell_ros2_node executable.
// When LoadCellNode is composed into turret_ros2_node, only the class above
// is linked in; this main() is NOT included in that build.
// ---------------------------------------------------------------------------
#ifdef LOAD_CELL_STANDALONE_MAIN

static std::shared_ptr<LoadCellNode> g_lc_node;

static void standaloneSignalHandler(int /*signum*/)
{
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return 1;
    }

    signal(SIGINT, standaloneSignalHandler);
    signal(SIGTERM, standaloneSignalHandler);

    try {
        g_lc_node = std::make_shared<LoadCellNode>();
        rclcpp::spin(g_lc_node);
    } catch (const std::exception& e) {
        std::cerr << "LoadCellNode error: " << e.what() << std::endl;
    }

    g_lc_node.reset();
    gpioTerminate();
    rclcpp::shutdown();
    return 0;
}

#endif // LOAD_CELL_STANDALONE_MAIN

#ifndef LOAD_CELL_NODE_H
#define LOAD_CELL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "turret_control/msg/load_cell_force.hpp"
#include "load_cell.h"
#include <memory>

/**
 * LoadCellNode – standalone ROS2 node for HX711-based load cells.
 *
 * Runs as a separate node (and optionally a separate process) from
 * TurretROS2Node.  When composed into the same process with
 * TurretROS2Node via MultiThreadedExecutor, the slow bit-banging I/O
 * of the HX711 chips runs on its own executor thread and cannot block
 * the turret control loop.
 *
 * Publishes: turret{id}/load_cell_force  (LoadCellForce, 50 Hz)
 * Services:  turret{id}/tare_load_cells  (Trigger)
 */
class LoadCellNode : public rclcpp::Node
{
public:
    LoadCellNode();
    ~LoadCellNode();

private:
    std::string topic_prefix_;
    std::unique_ptr<LoadCell> load_cell_;
    bool load_cell_ready_{false};

    rclcpp::Publisher<turret_control::msg::LoadCellForce>::SharedPtr load_cell_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    int readTurretId();
    void initializeLoadCells(int turret_id);
    void publishCallback();
    void tareCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif // LOAD_CELL_NODE_H

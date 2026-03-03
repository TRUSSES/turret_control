#ifndef LOAD_CELL_H
#define LOAD_CELL_H

#include <string>
#include <vector>
#include <utility>
#include <memory>
#include "hx711.h"

class LoadCell {
public:
    // Static method to reset GPIO pins before initializing HX711 hardware
    // Call this before creating LoadCell object to ensure clean state
    static void ResetGPIOPins();

    // Constructor: initializes HX711 objects using default GPIO pins and gain.
    LoadCell();
    ~LoadCell();

    // Runs interactive calibration. The user is prompted for a number of calibration points.
    // Returns true if calibration is successful.
    bool calibrate(int numCalPoints);

    // Loads calibration data from a config file (default "global_calibration.cfg").
    bool loadCalibrationData(const std::string& configFile = "global_calibration.cfg");

    // Saves calibration data to a config file (default "global_calibration.cfg").
    bool saveCalibrationData(const std::string& configFile = "global_calibration.cfg");

    // Re-tare the load cells.
    bool tare();

    // Returns the current force measurement.
    // If unit=="N" (or "n") the force is returned in newtons;
    // otherwise, the value is in grams.
    double getForce(const std::string& unit = "g");

private:
    // Three HX711 objects. Any chip that fails to initialise is left as nullptr
    // and silently skipped in all read/tare/calibrate operations.
    std::unique_ptr<HX711> hx1;
    std::unique_ptr<HX711> hx2;
    std::unique_ptr<HX711> hx3;
    int active_chips_{0};  // count of chips that initialised successfully

    // Global calibration parameters for the summed (tared) readings.
    double global_m;
    double global_b;

    // Converts a weight in grams to newtons.
    double gramsToNewtons(double grams);

    // Performs linear regression on the given data points.
    // Returns a pair (m, b) for the equation: weight = m * (tared_sum) + b.
    std::pair<double, double> linearRegression(const std::vector<double>& x, const std::vector<double>& y);
};

#endif // LOAD_CELL_CONTROLLER_H

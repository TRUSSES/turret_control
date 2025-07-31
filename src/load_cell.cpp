#include "load_cell.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <stdexcept>

// Constructor: initialize HX711 objects with your default wiring.
LoadCell::LoadCell()
    : hx1(17, 27, 128, 'A'),
      hx2(22, 27, 128, 'A'),
      hx3(24, 27, 128, 'A'),
      global_m(0.0),
      global_b(0.0)
{
    // Any additional initialization if needed.
}

// Destructor.
LoadCell::~LoadCell() {
    // Nothing specific to clean up.
}

// Tare the load cells.
bool LoadCell::tare() {
    std::cout << "Taring load cells..." << std::endl;
    if (!hx1.zero(30))
        std::cout << "Load Cell 1 tare successful." << std::endl;
    else
        std::cout << "Load Cell 1 tare error." << std::endl;

    if (!hx2.zero(30))
        std::cout << "Load Cell 2 tare successful." << std::endl;
    else
        std::cout << "Load Cell 2 tare error." << std::endl;

    if (!hx3.zero(30))
        std::cout << "Load Cell 3 tare successful." << std::endl;
    else
        std::cout << "Load Cell 3 tare error." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

// Linear regression on the calibration data.
std::pair<double, double> LoadCell::linearRegression(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.empty()) {
        throw std::runtime_error("Invalid calibration data");
    }
    double n = x.size();
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (size_t i = 0; i < x.size(); i++) {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_x2 += x[i] * x[i];
    }
    double denominator = (n * sum_x2 - sum_x * sum_x);
    if (denominator == 0) {
        throw std::runtime_error("Denominator in linear regression is zero");
    }
    double m = (n * sum_xy - sum_x * sum_y) / denominator;
    double b = (sum_y - m * sum_x) / n;
    return std::make_pair(m, b);
}

// Interactive calibration routine.
bool LoadCell::calibrate(int numCalPoints) {
    std::cout << "Starting calibration. Make sure no weight is on the plate." << std::endl;
    std::cout << "Press Enter to tare the load cells..." << std::endl;
    std::cin.ignore();
    tare();

    std::vector<double> global_tared;    // Sums of tared readings.
    std::vector<double> global_weight;     // Known weights in grams.

    for (int i = 0; i < numCalPoints; i++) {
        std::cout << "\nCalibration point " << (i+1) << ": Place a known weight on the plate and press Enter..." << std::endl;
        std::cin.ignore();
        std::cout << "Enter the known total weight (in grams): ";
        double known_weight;
        std::cin >> known_weight;
        std::cin.ignore();

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int tared1 = hx1.get_data_mean(5);
        int tared2 = hx2.get_data_mean(5);
        int tared3 = hx3.get_data_mean(5);
        int sum_tared = tared1 + tared2 + tared3;

        std::cout << "Calibration point " << (i+1) 
                  << ": tared sum = " << sum_tared 
                  << ", known weight = " << known_weight << std::endl;

        global_tared.push_back(sum_tared);
        global_weight.push_back(known_weight);
    }

    auto reg = linearRegression(global_tared, global_weight);
    global_m = reg.first;
    global_b = reg.second;

    std::cout << "\nCalibration complete: m = " << global_m << ", b = " << global_b << std::endl;
    return saveCalibrationData();
}

// Save calibration data to a file.
bool LoadCell::saveCalibrationData(const std::string& configFile) {
    std::ofstream config(configFile);
    if (!config) {
        std::cerr << "Error opening config file for writing!" << std::endl;
        return false;
    }
    config << global_m << " " << global_b << std::endl;
    config.close();
    std::cout << "Calibration data saved to " << configFile << std::endl;
    return true;
}

// Load calibration data from a file.
bool LoadCell::loadCalibrationData(const std::string& configFile) {
    std::ifstream config(configFile);
    if (!config) {
        std::cerr << "Error: Calibration config file not found." << std::endl;
        return false;
    }
    config >> global_m >> global_b;
    config.close();
    std::cout << "Calibration data loaded: m = " << global_m << ", b = " << global_b << std::endl;
    return true;
}

// Convert grams to newtons.
double LoadCell::gramsToNewtons(double grams) {
    return grams * 0.00981;
}

// Get the current force measurement.
// Uses the global calibration equation on the tared sum of readings.
// Returns the value in grams by default; if unit=="N" (or "n") returns newtons.
double LoadCell::getForce(const std::string& unit) {
    int tared1 = hx1.get_data_mean(5);
    int tared2 = hx2.get_data_mean(5);
    int tared3 = hx3.get_data_mean(5);
    int sum_tared = tared1 + tared2 + tared3;
    
    // Debug: print intermediate values
    std::cout << "[DEBUG] Tared readings: " 
              << tared1 << ", " << tared2 << ", " << tared3 
              << " | Sum: " << sum_tared << std::endl;
    
    double total_weight = global_m * sum_tared + global_b; // in grams

    if (unit == "N" || unit == "n") {
        double force_newtons = gramsToNewtons(total_weight);
        std::cout << "[DEBUG] Using calibration: m = " << global_m 
                  << ", b = " << global_b 
                  << " | Calculated weight (g): " << total_weight 
                  << " => Force (N): " << force_newtons << std::endl;
        return force_newtons;
    }
    return total_weight;
}


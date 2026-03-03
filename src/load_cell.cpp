#include "load_cell.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <pigpio.h>

// Static method to reset GPIO pins before initializing HX711 hardware
void LoadCell::ResetGPIOPins() {
    // HX711 GPIO pins used:
    // HX1: DOUT=22, PD_SCK=27
    // HX2: DOUT=24, PD_SCK=27 (shared)
    // HX3: DOUT=25, PD_SCK=27 (shared)

    std::cout << "LoadCell: Resetting GPIO pins for HX711..." << std::endl;

    // Set all pins to input mode to release them
    gpioSetMode(22, PI_INPUT);
    gpioSetMode(24, PI_INPUT);
    gpioSetMode(25, PI_INPUT);
    gpioSetMode(27, PI_INPUT);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Now set PD_SCK as output and cycle it to reset HX711 chips
    gpioSetMode(27, PI_OUTPUT);

    // Pull PD_SCK high for a bit (power down state)
    gpioWrite(27, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Pull PD_SCK low (power up and reset)
    gpioWrite(27, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Set DOUT pins as inputs (HX711 drives them)
    gpioSetMode(22, PI_INPUT);
    gpioSetMode(24, PI_INPUT);
    gpioSetMode(25, PI_INPUT);

    std::cout << "LoadCell: GPIO pins reset complete" << std::endl;
}

// Constructor: initialize HX711 objects with your default wiring.
// Each chip is initialised independently; failed chips are left as nullptr
// and skipped in all subsequent operations so the remaining chips still work.
LoadCell::LoadCell()
    : global_m(0.0),
      global_b(0.0)
{
    struct ChipCfg { int dout; const char* name; std::unique_ptr<HX711>& slot; };
    ChipCfg chips[] = {
        {22, "HX1", hx1},
        {24, "HX2", hx2},
        {25, "HX3", hx3},
    };
    for (auto& c : chips) {
        try {
            c.slot = std::make_unique<HX711>(c.dout, 27, 128, 'A');
            std::cout << "LoadCell: " << c.name << " (DOUT=" << c.dout << ") OK" << std::endl;
            ++active_chips_;
        } catch (const std::exception& e) {
            std::cerr << "LoadCell: " << c.name << " (DOUT=" << c.dout
                      << ") failed: " << e.what() << " – skipping" << std::endl;
            c.slot = nullptr;
        }
    }
    if (active_chips_ == 0) {
        throw std::runtime_error("LoadCell: all HX711 chips failed to initialise");
    }
    std::cout << "LoadCell: " << active_chips_ << "/3 chips active" << std::endl;
}

// Destructor.
LoadCell::~LoadCell() {
    // Power down all HX711 chips to release GPIO pins
    for (auto* chip : {hx1.get(), hx2.get(), hx3.get()}) {
        if (!chip) continue;
        try { chip->power_down(); } catch (...) {}
    }
    std::cout << "LoadCell: chips powered down" << std::endl;
}

// Tare the load cells.
bool LoadCell::tare() {
    std::cout << "Taring load cells..." << std::endl;
    const char* names[] = {"HX1", "HX2", "HX3"};
    HX711* chips[] = {hx1.get(), hx2.get(), hx3.get()};
    for (int i = 0; i < 3; ++i) {
        if (!chips[i]) { std::cout << names[i] << ": skipped (not available)" << std::endl; continue; }
        if (!chips[i]->zero(30))
            std::cout << names[i] << ": tare successful" << std::endl;
        else
            std::cout << names[i] << ": tare error" << std::endl;
    }

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

        HX711* chips[] = {hx1.get(), hx2.get(), hx3.get()};
        int sum_tared = 0; int n_read = 0;
        for (auto* c : chips) { if (c) { sum_tared += c->get_data_mean(5); ++n_read; } }
        if (n_read > 0 && n_read < 3) sum_tared = sum_tared * 3 / n_read;

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
    return 0;;
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
    std::cout << "Attempting to load calibration from: " << configFile << std::endl;
    std::ifstream config(configFile);
    if (!config) {
        std::cout << "Error: Calibration config file not found: " << configFile << std::endl;
        return false;
    }
    config >> global_m >> global_b;
    config.close();
    std::cout << "Calibration data loaded successfully: m = " << global_m << ", b = " << global_b << std::endl;
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
    // 1 sample per chip: one HX711 conversion at 10 SPS takes ~100 ms.
    // Using 1 sample per chip × 3 chips = ~300 ms per getForce() call.
    // Increasing samples improves noise rejection but slows the publish rate proportionally.
    static constexpr int kSamplesPerChip = 1;
    HX711* chips[] = {hx1.get(), hx2.get(), hx3.get()};
    int sum_tared = 0; int n_read = 0;
    for (auto* c : chips) { if (c) { sum_tared += c->get_data_mean(kSamplesPerChip); ++n_read; } }
    if (n_read == 0) return 0.0;
    // Normalise to 3-chip equivalent so calibration constants remain valid
    if (n_read < 3) sum_tared = sum_tared * 3 / n_read;

    double total_weight = global_m * sum_tared + global_b; // in grams

    if (unit == "N" || unit == "n") {
        double force_newtons = gramsToNewtons(total_weight);
        // std::cout << "[DEBUG] Using calibration: m = " << global_m 
        //           << ", b = " << global_b 
        //           << " | Calculated weight (g): " << total_weight 
        //           << " => Force (N): " << force_newtons << std::endl;
        return force_newtons;
    }
    return total_weight;
}


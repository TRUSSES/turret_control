#include <iostream>
#include <pigpio.h>
#include "load_cell.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: calibrate_load_cells <output_config_file>" << std::endl;
        return 1;
    }

    if (gpioInitialise() < 0) {
    std::cerr << "pigpio initialization failed" << std::endl;
    exit(1);
  }

    std::string output_file = argv[1];

    std::cout << "Initializing load cell..." << std::endl;
    LoadCell load_cell;
    
    std::cout << "Taring the scale. Make sure nothing is on it..." << std::endl;
    if (!load_cell.tare()) {
        std::cerr << "Tare failed!" << std::endl;
        return 1;
    }
    std::cout << "Tare successful.\n" << std::endl;

    int num_points;
    std::cout << "Enter number of calibration points: ";
    std::cin >> num_points;

    if (!load_cell.calibrate(num_points)) {
        std::cerr << "Calibration failed!" << std::endl;
        return 1;
    }

    if (!load_cell.saveCalibrationData(output_file)) {
        std::cerr << "Failed to save calibration data to " << output_file << std::endl;
        return 1;
    }

    std::cout << "\n✅ Calibration completed and saved to " << output_file << std::endl;
    return 0;
}

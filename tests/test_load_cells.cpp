/**
 * test_load_cells.cpp
 *
 * Standalone diagnostic for HX711-based load cells.
 * Run this INSTEAD of the full turret node to isolate load cell issues.
 *
 * Build:  colcon build  (added as test_load_cells target in CMakeLists.txt)
 * Run:    sudo ./install/turret_control/lib/turret_control/test_load_cells
 *
 * What it tests:
 *   1. Raw DOUT pin state before any clocking (should be LOW if chips are alive)
 *   2. Manual GPIO power-cycle and chip response
 *   3. Raw bit-bang read from each chip individually
 *   4. Full LoadCell class init and calibrated reading
 *   5. Continuous reading loop at 10 Hz for 30 s to spot intermittent failures
 */

#include <pigpio.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include "hx711.h"
#include "load_cell.h"

// ── GPIO pin assignments (must match load_cell.cpp) ────────────────────────
static const int SCK   = 27;
static const int DOUT1 = 22;   // HX1
static const int DOUT2 = 24;   // HX2
static const int DOUT3 = 25;   // HX3

static volatile bool g_running = true;

static void sigHandler(int) { g_running = false; }

// ── helpers ─────────────────────────────────────────────────────────────────
static void sep(const std::string& title) {
    std::cout << "\n══════════════════════════════════════════════\n";
    std::cout << "  " << title << "\n";
    std::cout << "══════════════════════════════════════════════\n";
}

static void sleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Read DOUT pin state and return the level (0 = LOW/ready, 1 = HIGH/not-ready)
static int readDout(int pin) { return gpioRead(pin); }

// Manual power-cycle: SCK HIGH >60 µs → power-down, then SCK LOW → power-up
static void manualPowerCycle() {
    gpioSetMode(SCK, PI_OUTPUT);
    gpioWrite(SCK, 0);
    sleepMs(5);
    gpioWrite(SCK, 1);   // power-down
    sleepMs(100);
    gpioWrite(SCK, 0);   // power-up
    sleepMs(10);
}

// Wait up to timeoutMs for DOUT pin to go LOW; return true if it went LOW.
static bool waitReady(int pin, int timeoutMs) {
    for (int i = 0; i < timeoutMs; ++i) {
        if (gpioRead(pin) == 0) return true;
        sleepMs(1);
    }
    return false;
}

// Manual 25-pulse bit-bang read from one DOUT pin (shared SCK).
// Returns true and fills rawValue on success.
static bool manualRead(int dout, int& rawValue) {
    // Wait up to 500 ms for chip to be ready
    if (!waitReady(dout, 500)) {
        std::cout << "    DOUT (GPIO " << dout << ") never went LOW after 500 ms\n";
        return false;
    }
    // Clock out 24 data bits
    unsigned long data = 0;
    for (int i = 0; i < 24; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();
        gpioWrite(SCK, 1);
        gpioWrite(SCK, 0);
        auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - t0).count();
        data = (data << 1) | gpioRead(dout);
        if (dt >= 60) {
            std::cout << "    WARNING: clock pulse " << i << " took " << dt
                      << " µs (>= 60 µs – chip may have entered power-down!)\n";
        }
    }
    // 25th pulse – sets gain A/128 for next conversion
    gpioWrite(SCK, 1);
    gpioWrite(SCK, 0);

    // Two's-complement decode
    if (data & 0x800000)
        rawValue = -static_cast<int>(((~data) & 0xFFFFFF) + 1);
    else
        rawValue = static_cast<int>(data);

    return true;
}

// ── TEST 1: raw pin states ───────────────────────────────────────────────────
static void test1_rawPinStates() {
    sep("TEST 1 – Raw DOUT pin states (no clocking)");
    std::cout << "  All DOUT pins should be LOW if chips are powered and idle.\n\n";

    // Set SCK LOW, DOUT pins as inputs
    gpioSetMode(SCK,   PI_OUTPUT); gpioWrite(SCK, 0);
    gpioSetMode(DOUT1, PI_INPUT);
    gpioSetMode(DOUT2, PI_INPUT);
    gpioSetMode(DOUT3, PI_INPUT);
    sleepMs(100);

    auto report = [](const char* name, int pin) {
        int level = gpioRead(pin);
        std::cout << "  GPIO " << std::setw(2) << pin
                  << " (" << name << "): "
                  << (level == 0 ? "LOW  ✓ (chip ready)" : "HIGH ✗ (chip NOT ready)")
                  << "\n";
    };
    report("DOUT1/HX1", DOUT1);
    report("DOUT2/HX2", DOUT2);
    report("DOUT3/HX3", DOUT3);
}

// ── TEST 2: pin states after power-cycle ────────────────────────────────────
static void test2_afterPowerCycle() {
    sep("TEST 2 – DOUT pin states after manual power-cycle + 2 s wait");

    manualPowerCycle();
    std::cout << "  Power-cycled. Waiting 2 s...\n";
    sleepMs(2000);

    auto report = [](const char* name, int pin, int waitMs) {
        int level = gpioRead(pin);
        std::cout << "  GPIO " << std::setw(2) << pin
                  << " (" << name << ") at " << waitMs << " ms: "
                  << (level == 0 ? "LOW  ✓" : "HIGH ✗")
                  << "\n";
    };
    report("DOUT1/HX1", DOUT1, 2000);
    report("DOUT2/HX2", DOUT2, 2000);
    report("DOUT3/HX3", DOUT3, 2000);

    std::cout << "  Waiting another 8 s (10 s total)...\n";
    sleepMs(8000);
    std::cout << "  At 10 s:\n";
    report("DOUT1/HX1", DOUT1, 10000);
    report("DOUT2/HX2", DOUT2, 10000);
    report("DOUT3/HX3", DOUT3, 10000);
}

// ── TEST 3: manual bit-bang read from each chip ──────────────────────────────
static void test3_manualRead() {
    sep("TEST 3 – Manual bit-bang read from each chip individually");
    std::cout << "  (SCK is shared; reading one chip clocks all three)\n\n";

    struct ChipInfo { const char* name; int dout; };
    ChipInfo chips[] = {
        {"HX1 (DOUT=GPIO22)", DOUT1},
        {"HX2 (DOUT=GPIO24)", DOUT2},
        {"HX3 (DOUT=GPIO25)", DOUT3},
    };

    // Do a fresh power-cycle first
    manualPowerCycle();
    std::cout << "  Power-cycled. Waiting 10 s for stabilization...\n";
    for (int i = 10; i > 0; --i) {
        std::cout << "  " << i << "...\r" << std::flush;
        sleepMs(1000);
    }
    std::cout << "\n";

    for (auto& c : chips) {
        std::cout << "  Reading " << c.name << ":\n";
        for (int attempt = 1; attempt <= 3; ++attempt) {
            int raw = 0;
            bool ok = manualRead(c.dout, raw);
            if (ok) {
                std::cout << "    Attempt " << attempt << ": raw = " << raw
                          << (raw == 0 ? "  (all zeros – chip may be disconnected)" : "")
                          << "\n";
            } else {
                std::cout << "    Attempt " << attempt << ": FAILED\n";
            }
            sleepMs(200);  // give chip time for next conversion
        }
    }
}

// ── TEST 4: LoadCell class with debug mode ───────────────────────────────────
static void test4_loadCellClass() {
    sep("TEST 4 – LoadCell class (enables HX711 debug mode)");

    std::cout << "  Calling LoadCell::ResetGPIOPins()...\n";
    LoadCell::ResetGPIOPins();

    std::cout << "  Waiting 10 s for chips to stabilize...\n";
    for (int i = 10; i > 0; --i) {
        std::cout << "  " << i << "...\r" << std::flush;
        sleepMs(1000);
    }
    std::cout << "\n";

    std::cout << "  Creating LoadCell object...\n";
    LoadCell lc;
    std::cout << "  LoadCell created successfully.\n";

    // Try loading calibration
    std::string cfg = "/home/turret/ros_ws/src/turret_control/config/turret_1_lc_config.cfg";
    bool calibrated = lc.loadCalibrationData(cfg);
    if (calibrated) {
        std::cout << "  Calibration loaded from: " << cfg << "\n";
    } else {
        std::cout << "  No calibration file found – will read raw (un-calibrated) force.\n";
        std::cout << "  (Run calibrate_load_cells to generate a calibration file)\n";
    }

    std::cout << "\n  Reading force for 10 s (Ctrl-C to stop early)...\n\n";
    int count = 0;
    while (g_running && count < 100) {
        try {
            double force_N = lc.getForce("N");
            double force_g = lc.getForce("g");
            std::cout << "  [" << std::setw(3) << count << "] "
                      << std::fixed << std::setprecision(3)
                      << force_N << " N  (" << force_g << " g)\n";
        } catch (const std::exception& e) {
            std::cout << "  [" << std::setw(3) << count << "] ERROR: " << e.what() << "\n";
        }
        ++count;
        sleepMs(100);
    }
}

// ── TEST 5: clock-pulse timing diagnostic ───────────────────────────────────
static void test5_clockTiming() {
    sep("TEST 5 – Clock pulse timing (checks if OS preemption causes >60 µs pulses)");
    std::cout << "  Sending 100 clock pulses and measuring HIGH time for each.\n";
    std::cout << "  Any pulse >=60 µs puts the HX711 into power-down mode.\n\n";

    gpioSetMode(SCK, PI_OUTPUT);
    gpioWrite(SCK, 0);
    sleepMs(10);

    int over60 = 0, over30 = 0;
    long maxUs = 0;
    for (int i = 0; i < 100; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        gpioWrite(SCK, 1);
        gpioWrite(SCK, 0);
        long us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - t0).count();
        if (us > maxUs) maxUs = us;
        if (us >= 60) { ++over60; std::cout << "  Pulse " << i << ": " << us << " µs  ← DANGER\n"; }
        else if (us >= 30) { ++over30; std::cout << "  Pulse " << i << ": " << us << " µs  (marginal)\n"; }
        sleepMs(2);
    }
    std::cout << "\n  Results:\n";
    std::cout << "    Max pulse HIGH time : " << maxUs << " µs\n";
    std::cout << "    Pulses >= 60 µs     : " << over60 << " / 100  (DANGEROUS – chip enters power-down)\n";
    std::cout << "    Pulses >= 30 µs     : " << over30 << " / 100  (marginal)\n";
    if (over60 == 0)
        std::cout << "    ✓ Clock timing OK for this process (no dangerous pulses)\n";
    else
        std::cout << "    ✗ OS scheduling is too coarse – bit-banging unreliable on this system\n";
}

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* /*argv*/[]) {
    (void)argc;
    signal(SIGINT,  sigHandler);
    signal(SIGTERM, sigHandler);

    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║       HX711 Load Cell Diagnostic Tool            ║\n";
    std::cout << "║  Pins: SCK=GPIO27  HX1=GPIO22  HX2=GPIO24  HX3=GPIO25 ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n";

    std::cout << "\nInitialising pigpio...\n";
    if (gpioInitialise() < 0) {
        std::cerr << "ERROR: pigpio initialisation failed. Run as root (sudo).\n";
        return 1;
    }
    std::cout << "pigpio OK\n";

    // Run all tests in sequence
    test1_rawPinStates();
    test2_afterPowerCycle();
    test3_manualRead();
    test5_clockTiming();
    if (g_running) test4_loadCellClass();

    sep("DONE");
    std::cout << "  Check the output above for ✗ markers.\n";
    std::cout << "  If DOUT pins are always HIGH: wiring or power issue.\n";
    std::cout << "  If DOUT goes LOW but reads are 0: check DOUT wire to each chip.\n";
    std::cout << "  If timing pulses are >=60µs: OS jitter is causing power-downs.\n\n";

    gpioTerminate();
    return 0;
}

#include "hx711.h"
#include <pigpio.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <bitset>  // Added for std::bitset

using namespace std::chrono;

// Constructor – initializes internal variables and configures the GPIO pins.
HX711::HX711(int dout_pin, int pd_sck_pin, int gain_channel_A, char channel)
    : _pd_sck(pd_sck_pin), _dout(dout_pin),
      _gain_channel_A(gain_channel_A),
      _offset_A_128(0), _offset_A_64(0), _offset_B(0),
      _last_raw_data_A_128(0), _last_raw_data_A_64(0), _last_raw_data_B(0),
      _wanted_channel('A'), _current_channel('A'),
      _scale_ratio_A_128(1.0), _scale_ratio_A_64(1.0), _scale_ratio_B(1.0),
      _debug_mode(false)
{
    // Set pin modes using pigpio functions.
    gpioSetMode(_pd_sck, PI_OUTPUT);
    gpioSetMode(_dout, PI_INPUT);

    // Set default data filter using a lambda that calls outliers_filter with default threshold.
    _data_filter = [this](const std::vector<int>& data) -> std::vector<int> {
        return this->outliers_filter(data, 1.0);
    };

    // Use the provided channel parameter.
    select_channel(channel);
    set_gain_A(gain_channel_A);
}

HX711::~HX711() {
    // Optionally power down the chip here.
}

void HX711::select_channel(char channel) {
    char ch = toupper(channel);
    if (ch == 'A' || ch == 'B') {
        _wanted_channel = ch;
    } else {
        throw std::invalid_argument("select_channel: channel must be 'A' or 'B'");
    }
    // After changing channel, perform a reading and allow settling.
    _read();
    std::this_thread::sleep_for(milliseconds(500));
}

void HX711::set_gain_A(int gain) {
    if (gain == 128 || gain == 64) {
        _gain_channel_A = gain;
    } else {
        throw std::invalid_argument("set_gain_A: gain must be 128 or 64");
    }
    // After changing gain, perform a reading and allow settling.
    _read();
    std::this_thread::sleep_for(milliseconds(500));
}

bool HX711::zero(int readings) {
    if (readings <= 0 || readings >= 100) {
        throw std::invalid_argument("zero: readings must be between 1 and 99");
    }
    int result = get_raw_data_mean(readings);
    if (_current_channel == 'A' && _gain_channel_A == 128) {
        _offset_A_128 = result;
    } else if (_current_channel == 'A' && _gain_channel_A == 64) {
        _offset_A_64 = result;
    } else if (_current_channel == 'B') {
        _offset_B = result;
    } else {
        if (_debug_mode)
            std::cout << "zero(): Channel and gain mismatch. current channel: "
                      << _current_channel << " gain: " << _gain_channel_A << std::endl;
        return true;
    }
    return false;
}

void HX711::set_offset(int offset, char channel, int gain_A) {
    if (channel != '\0') {
        char ch = toupper(channel);
        if (ch == 'A' && gain_A == 128)
            _offset_A_128 = offset;
        else if (ch == 'A' && gain_A == 64)
            _offset_A_64 = offset;
        else if (ch == 'B')
            _offset_B = offset;
        else
            throw std::invalid_argument("set_offset: channel must be 'A' or 'B'");
    } else {
        if (_current_channel == 'A' && _gain_channel_A == 128)
            _offset_A_128 = offset;
        else if (_current_channel == 'A' && _gain_channel_A == 64)
            _offset_A_64 = offset;
        else
            _offset_B = offset;
    }
}

void HX711::set_scale_ratio(double scale_ratio, char channel, int gain_A) {
    if (channel != '\0') {
        char ch = toupper(channel);
        if (ch == 'A' && gain_A == 128)
            _scale_ratio_A_128 = scale_ratio;
        else if (ch == 'A' && gain_A == 64)
            _scale_ratio_A_64 = scale_ratio;
        else if (ch == 'B')
            _scale_ratio_B = scale_ratio;
        else
            throw std::invalid_argument("set_scale_ratio: channel must be 'A' or 'B'");
    } else {
        if (_current_channel == 'A' && _gain_channel_A == 128)
            _scale_ratio_A_128 = scale_ratio;
        else if (_current_channel == 'A' && _gain_channel_A == 64)
            _scale_ratio_A_64 = scale_ratio;
        else
            _scale_ratio_B = scale_ratio;
    }
}

void HX711::set_data_filter(std::function<std::vector<int>(const std::vector<int>&)> data_filter) {
    if (data_filter) {
        _data_filter = data_filter;
    } else {
        throw std::invalid_argument("set_data_filter: data_filter must be callable");
    }
}

void HX711::set_debug_mode(bool flag) {
    _debug_mode = flag;
    std::cout << "Debug mode " << (_debug_mode ? "ENABLED" : "DISABLED") << std::endl;
}

void HX711::_save_last_raw_data(char channel, int gain_A, int data) {
    if (channel == 'A' && gain_A == 128)
        _last_raw_data_A_128 = data;
    else if (channel == 'A' && gain_A == 64)
        _last_raw_data_A_64 = data;
    else if (channel == 'B')
        _last_raw_data_B = data;
}

bool HX711::_ready() {
    return (gpioRead(_dout) == 0);
}

bool HX711::_set_channel_gain(int num) {
    for (int i = 0; i < num; i++) {
        auto start = high_resolution_clock::now();
        gpioWrite(_pd_sck, 1);
        gpioWrite(_pd_sck, 0);
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end - start).count();
        if (duration >= 60) {
            if (_debug_mode) {
                std::cout << "Not fast enough while setting gain/channel. Time elapsed: " 
                          << duration << " us" << std::endl;
            }
            try {
                get_raw_data_mean(6);
            } catch (...) {
                return false;
            }
        }
    }
    return true;
}

int HX711::_read() {
    gpioWrite(_pd_sck, 0);
    int ready_counter = 0;
    // HX711 at 10 SPS has a 100 ms conversion period. Allow 5 full cycles (500 ms)
    // to tolerate Linux scheduling jitter and post-reset startup delays.
    int max_iterations = 500;
    while (!_ready() && ready_counter <= max_iterations) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ready_counter++;
    }
    if (ready_counter > max_iterations) {
        if (_debug_mode)
            std::cout << "_read() not ready after " << ready_counter << " trials" << std::endl;
        throw std::runtime_error("_read: HX711 not ready");
    }
    
    unsigned long data_in = 0;
    for (int i = 0; i < 24; i++) {
        auto start = high_resolution_clock::now();
        gpioWrite(_pd_sck, 1);
        gpioWrite(_pd_sck, 0);
        auto end = high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        if (duration >= 60) {
            if (_debug_mode) {
                std::cout << "Not fast enough while reading data. Time elapsed: " 
                          << duration << " us" << std::endl;
            }
            throw std::runtime_error("_read: Timing error while reading data");
        }
        data_in = (data_in << 1) | gpioRead(_dout);
    }
    
    if (_wanted_channel == 'A' && _gain_channel_A == 128) {
        if (!_set_channel_gain(1))
            throw std::runtime_error("_read: Failed to set channel/gain");
        _current_channel = 'A';
        _gain_channel_A = 128;
    } else if (_wanted_channel == 'A' && _gain_channel_A == 64) {
        if (!_set_channel_gain(3))
            throw std::runtime_error("_read: Failed to set channel/gain");
        _current_channel = 'A';
        _gain_channel_A = 64;
    } else {
        if (!_set_channel_gain(2))
            throw std::runtime_error("_read: Failed to set channel/gain");
        _current_channel = 'B';
    }
    
    if (_debug_mode) {
        std::bitset<24> bs(data_in);
        std::cout << "Raw binary value: " << bs << std::endl;
    }
    
    if (data_in == 0x7FFFFF || data_in == 0x800000) {
        if (_debug_mode)
            std::cout << "Invalid data detected: " << data_in << std::endl;
        throw std::runtime_error("_read: Invalid data detected");
    }
    
    int signed_data = 0;
    if (data_in & 0x800000) {
        signed_data = -static_cast<int>(((~data_in) & 0xFFFFFF) + 1);
    } else {
        signed_data = static_cast<int>(data_in);
    }
    
    if (_debug_mode) {
        std::cout << "Converted value: " << signed_data << std::endl;
    }
    return signed_data;
}


int HX711::get_raw_data_mean(int readings) {
    char backup_channel = _current_channel;
    int backup_gain = _gain_channel_A;
    std::vector<int> data_list;
    for (int i = 0; i < readings; i++) {
        data_list.push_back(_read());
    }
    double data_mean = 0.0;
    if (readings > 2 && _data_filter) {
        std::vector<int> filtered_data = _data_filter(data_list);
        if (filtered_data.empty()) {
            throw std::runtime_error("get_raw_data_mean: Data filter removed all data");
        }
        if (_debug_mode) {
            std::cout << "Raw data readings:";
            for (auto v : data_list)
                std::cout << " " << v;
            std::cout << "\nFiltered data:";
            for (auto v : filtered_data)
                std::cout << " " << v;
            std::cout << std::endl;
        }
        data_mean = calculate_mean(filtered_data);
    } else {
        data_mean = calculate_mean(data_list);
    }
    _save_last_raw_data(backup_channel, backup_gain, static_cast<int>(data_mean));
    return static_cast<int>(data_mean);
}

int HX711::get_data_mean(int readings) {
    int result = get_raw_data_mean(readings);
    if (_current_channel == 'A' && _gain_channel_A == 128)
        return result - _offset_A_128;
    else if (_current_channel == 'A' && _gain_channel_A == 64)
        return result - _offset_A_64;
    else
        return result - _offset_B;
}

double HX711::get_weight_mean(int readings) {
    int result = get_raw_data_mean(readings);
    if (_current_channel == 'A' && _gain_channel_A == 128)
        return (result - _offset_A_128) / _scale_ratio_A_128;
    else if (_current_channel == 'A' && _gain_channel_A == 64)
        return (result - _offset_A_64) / _scale_ratio_A_64;
    else
        return (result - _offset_B) / _scale_ratio_B;
}

char HX711::get_current_channel() const {
    return _current_channel;
}

int HX711::get_current_gain_A() const {
    return _gain_channel_A;
}

int HX711::get_last_raw_data(char channel, int gain_A) const {
    char ch = (channel != '\0') ? toupper(channel) : _current_channel;
    if (ch == 'A' && (gain_A == 128 || (channel == '\0' && _gain_channel_A == 128)))
        return _last_raw_data_A_128;
    else if (ch == 'A' && (gain_A == 64 || (channel == '\0' && _gain_channel_A == 64)))
        return _last_raw_data_A_64;
    else if (ch == 'B')
        return _last_raw_data_B;
    else {
        throw std::invalid_argument("get_last_raw_data: Invalid channel/gain");
    }
}

int HX711::get_current_offset(char channel, int gain_A) const {
    char ch = (channel != '\0') ? toupper(channel) : _current_channel;
    if (ch == 'A' && (gain_A == 128 || (channel == '\0' && _gain_channel_A == 128)))
        return _offset_A_128;
    else if (ch == 'A' && (gain_A == 64 || (channel == '\0' && _gain_channel_A == 64)))
        return _offset_A_64;
    else if (ch == 'B')
        return _offset_B;
    else {
        throw std::invalid_argument("get_current_offset: Invalid channel/gain");
    }
}

double HX711::get_current_scale_ratio(char channel, int gain_A) const {
    char ch = (channel != '\0') ? toupper(channel) : _current_channel;
    if (ch == 'A' && (gain_A == 128 || (channel == '\0' && _gain_channel_A == 128)))
        return _scale_ratio_A_128;
    else if (ch == 'A' && (gain_A == 64 || (channel == '\0' && _gain_channel_A == 64)))
        return _scale_ratio_A_64;
    else if (ch == 'B')
        return _scale_ratio_B;
    else {
        throw std::invalid_argument("get_current_scale_ratio: Invalid channel/gain");
    }
}

void HX711::power_down() {
    gpioWrite(_pd_sck, 0);
    gpioWrite(_pd_sck, 1);
    std::this_thread::sleep_for(milliseconds(10));
}

void HX711::power_up() {
    gpioWrite(_pd_sck, 0);
    std::this_thread::sleep_for(milliseconds(10));
}

bool HX711::reset() {
    power_down();
    power_up();
    try {
        get_raw_data_mean(6);
    } catch (...) {
        return true;
    }
    return false;
}

std::vector<int> HX711::outliers_filter(const std::vector<int>& data_list, double stdev_thresh) {
    std::vector<int> data;
    for (int num : data_list) {
        if (num != -1)
            data.push_back(num);
    }
    if (data.empty())
        return {};

    double median = calculate_median(data);
    std::vector<double> dists;
    for (int v : data) {
        dists.push_back(std::abs(v - median));
    }
    double mean_dists = calculate_mean_double(dists);
    double stdev = calculate_stddev(dists, mean_dists);
    if (stdev == 0.0)
        return { static_cast<int>(median) };

    std::vector<int> filtered;
    for (size_t i = 0; i < data.size(); i++) {
        if ((dists[i] / stdev) < stdev_thresh)
            filtered.push_back(data[i]);
    }
    return filtered;
}

double HX711::calculate_mean(const std::vector<int>& data) {
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}

double HX711::calculate_median(std::vector<int> data) {
    std::sort(data.begin(), data.end());
    size_t n = data.size();
    if (n % 2 == 0)
        return (data[n / 2 - 1] + data[n / 2]) / 2.0;
    else
        return data[n / 2];
}

double HX711::calculate_stddev(const std::vector<double>& data, double mean) {
    double accum = 0.0;
    for (double d : data)
        accum += (d - mean) * (d - mean);
    return std::sqrt(accum / data.size());
}

double HX711::calculate_mean_double(const std::vector<double>& data) {
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}

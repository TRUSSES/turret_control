#ifndef HX711_H
#define HX711_H

#include <vector>
#include <functional>
#include <stdexcept>

class HX711 {
public:
    // Constructor and destructor
    HX711(int dout_pin, int pd_sck_pin, int gain_channel_A = 128, char channel = 'A');
    ~HX711();

    // Channel and gain configuration
    void select_channel(char channel);
    void set_gain_A(int gain);

    // Calibration (zero/tare) and configuration of offset/scale ratio
    // zero returns false on success; true if an error occurred.
    bool zero(int readings = 30);
    void set_offset(int offset, char channel = '\0', int gain_A = 0);
    void set_scale_ratio(double scale_ratio, char channel = '\0', int gain_A = 0);

    // Optionally, a user-defined data filter function (e.g. outlier filtering)
    void set_data_filter(std::function<std::vector<int>(const std::vector<int>&)> data_filter);

    // Debug mode (prints additional information if enabled)
    void set_debug_mode(bool flag);

    // Data reading functions
    int get_raw_data_mean(int readings = 30);
    int get_data_mean(int readings = 30);
    double get_weight_mean(int readings = 30);

    // Getters for internal state
    char get_current_channel() const;
    int get_current_gain_A() const;
    int get_last_raw_data(char channel = '\0', int gain_A = 0) const;
    int get_current_offset(char channel = '\0', int gain_A = 0) const;
    double get_current_scale_ratio(char channel = '\0', int gain_A = 0) const;

    // Power control and reset
    void power_down();
    void power_up();
    bool reset();

    // Outliers filter function: can be used as default data filter.
    std::vector<int> outliers_filter(const std::vector<int>& data_list, double stdev_thresh = 1.0);

private:
    // GPIO pins
    int _pd_sck;
    int _dout;

    // Gain, offset, scale, and last reading storage
    int _gain_channel_A;
    int _offset_A_128;
    int _offset_A_64;
    int _offset_B;
    int _last_raw_data_A_128;
    int _last_raw_data_A_64;
    int _last_raw_data_B;
    char _wanted_channel;
    char _current_channel;
    double _scale_ratio_A_128;
    double _scale_ratio_A_64;
    double _scale_ratio_B;

    // Debug flag and data filter function
    bool _debug_mode;
    std::function<std::vector<int>(const std::vector<int>&)> _data_filter;

    // Private helper functions
    void _save_last_raw_data(char channel, int gain_A, int data);
    bool _ready();
    bool _set_channel_gain(int num);
    int _read();

    // Statistical helper functions
    double calculate_mean(const std::vector<int>& data);
    double calculate_median(std::vector<int> data);
    double calculate_stddev(const std::vector<double>& data, double mean);
    double calculate_mean_double(const std::vector<double>& data);
};

#endif // HX711_H

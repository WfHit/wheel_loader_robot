#pragma once

#include "safety_types.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

namespace safety_manager
{

/**
 * @brief Base class for safety monitoring components
 *
 * Provides common functionality for all monitoring subsystems
 */
class SafetyMonitorBase
{
public:
    SafetyMonitorBase(const char* name) : _name(name) {}
    virtual ~SafetyMonitorBase() = default;

    virtual void update() = 0;
    virtual uint32_t get_faults() const { return _current_faults; }
    virtual float get_health_score() const { return _health_score; }
    virtual uint32_t get_violation_count() const { return _violation_count; }
    virtual void reset_violations() { _violation_count = 0; }

    const char* get_name() const { return _name; }

protected:
    void set_fault(FaultType fault) {
        safety_manager::set_fault(_current_faults, fault);
        _violation_count++;
        _last_fault_time = hrt_absolute_time();
        static hrt_abstime last_warn = 0;
        if (hrt_elapsed_time(&last_warn) > 5000000) { // 5 seconds
            PX4_WARN("%s: Fault detected - %d", _name, static_cast<int>(fault));
            last_warn = hrt_absolute_time();
        }
    }

    void clear_fault(FaultType fault) {
        safety_manager::clear_fault(_current_faults, fault);
    }

    void update_health_score(float score) {
        _health_score = math::constrain(score, 0.0f, 1.0f);
    }

    bool is_timeout(uint64_t last_time, uint64_t timeout_us) const {
        return (hrt_absolute_time() - last_time) > timeout_us;
    }

private:
    const char* _name;
    uint32_t _current_faults{static_cast<uint32_t>(FaultType::NONE)};
    uint32_t _violation_count{0};
    uint64_t _last_fault_time{0};
    float _health_score{1.0f};
};

/**
 * @brief Speed monitoring subsystem
 */
class SpeedMonitor : public SafetyMonitorBase
{
public:
    struct Config {
        float max_speed_ms{10.0f};
        float max_acceleration_ms2{3.0f};
        float emergency_decel_threshold{5.0f};
        float wheel_radius_m{DEFAULT_WHEEL_RADIUS_M};
    };

    struct State {
        float current_speed_ms{0.0f};
        float max_safe_speed_ms{0.0f};
        float acceleration_ms2{0.0f};
        bool speed_limit_exceeded{false};
        bool hard_braking_detected{false};
        uint64_t last_update_time{0};
    };

    SpeedMonitor() : SafetyMonitorBase("SpeedMonitor") {}

    void configure(const Config& config) { _config = config; }
    void update() override;
    void update_wheel_speed(float wheel_speed_rad_s);

    const State& get_state() const { return _state; }
    float get_current_speed() const { return _state.current_speed_ms; }
    bool is_speed_limit_exceeded() const { return _state.speed_limit_exceeded; }

private:
    Config _config;
    State _state;
    float _last_speed{0.0f};
    uint64_t _last_speed_time{0};

    void calculate_acceleration();
    void check_speed_limits();
    void update_safe_speed();
};

/**
 * @brief Steering monitoring subsystem
 */
class SteeringMonitor : public SafetyMonitorBase
{
public:
    struct Config {
        float max_angle_rad{1.0f}; // ~57 degrees
        float max_rate_rad_s{0.5f};
        float error_threshold_rad{0.1f}; // ~5.7 degrees
    };

    struct State {
        float current_angle_rad{0.0f};
        float commanded_angle_rad{0.0f};
        float steering_rate_rad_s{0.0f};
        bool angle_limit_exceeded{false};
        bool rate_limit_exceeded{false};
        bool steering_error_detected{false};
        uint64_t last_update_time{0};
    };

    SteeringMonitor() : SafetyMonitorBase("SteeringMonitor") {}

    void configure(const Config& config) { _config = config; }
    void update() override;
    void update_current_angle(float angle_rad);
    void update_commanded_angle(float angle_rad);

    const State& get_state() const { return _state; }
    bool is_angle_limit_exceeded() const { return _state.angle_limit_exceeded; }
    bool is_steering_error_detected() const { return _state.steering_error_detected; }

private:
    Config _config;
    State _state;
    float _last_angle{0.0f};
    uint64_t _last_angle_time{0};

    void calculate_steering_rate();
    void check_angle_limits();
    void check_rate_limits();
    void check_steering_error();
};

/**
 * @brief Stability monitoring subsystem
 */
class StabilityMonitor : public SafetyMonitorBase
{
public:
    struct Config {
        float max_roll_angle_rad{0.3f}; // ~17 degrees
        float max_pitch_angle_rad{0.3f}; // ~17 degrees
        float rollover_threshold{0.2f}; // Stability margin threshold
    };

    struct State {
        float roll_angle_rad{0.0f};
        float pitch_angle_rad{0.0f};
        float roll_rate_rad_s{0.0f};
        float pitch_rate_rad_s{0.0f};
        float lateral_acceleration_ms2{0.0f};
        float longitudinal_acceleration_ms2{0.0f};
        bool stability_warning{false};
        bool rollover_risk{false};
        bool tip_over_detected{false};
        float stability_margin{1.0f};
        uint64_t last_update_time{0};
    };

    StabilityMonitor() : SafetyMonitorBase("StabilityMonitor") {}

    void configure(const Config& config) { _config = config; }
    void update() override;
    void update_imu_data(float accel_x, float accel_y, float accel_z);
    void update_gyro_data(float gyro_x, float gyro_y, float gyro_z);

    const State& get_state() const { return _state; }
    bool is_stability_warning() const { return _state.stability_warning; }
    bool is_rollover_risk() const { return _state.rollover_risk; }
    float get_stability_margin() const { return _state.stability_margin; }

private:
    Config _config;
    State _state;

    void calculate_attitude_from_accel(float accel_x, float accel_y, float accel_z);
    void check_stability_limits();
    void calculate_stability_margin();
    void assess_rollover_risk();
};

/**
 * @brief Load monitoring subsystem
 */
class LoadMonitor : public SafetyMonitorBase
{
public:
    struct Config {
        float max_payload_kg{5000.0f};
        float max_cg_offset_m{1.0f};
        float load_shift_threshold_m{0.5f};
    };

    struct State {
        float payload_mass_kg{0.0f};
        float center_of_gravity_offset_m{0.0f};
        bool overload_detected{false};
        bool load_shift_detected{false};
        bool unsafe_load_distribution{false};
        uint64_t last_update_time{0};
    };

    LoadMonitor() : SafetyMonitorBase("LoadMonitor") {}

    void configure(const Config& config) { _config = config; }
    void update() override;
    void update_load_data(float payload_kg, float cg_offset_m);

    const State& get_state() const { return _state; }
    bool is_overload_detected() const { return _state.overload_detected; }
    bool is_unsafe_load_distribution() const { return _state.unsafe_load_distribution; }

private:
    Config _config;
    State _state;
    float _last_cg_offset{0.0f};

    void check_load_limits();
    void detect_load_shift();
};

/**
 * @brief Communication monitoring subsystem
 */
class CommunicationMonitor : public SafetyMonitorBase
{
public:
    struct Config {
        uint64_t timeout_us{COMMUNICATION_TIMEOUT_US};
    };

    struct State {
        uint64_t last_manual_command_time{0};
        uint64_t last_vehicle_command_time{0};
        uint64_t last_module_status_time{0};
        bool manual_control_timeout{false};
        bool vehicle_command_timeout{false};
        bool module_status_timeout{false};
        uint32_t communication_failures{0};
        float communication_quality{1.0f};
    };

    CommunicationMonitor() : SafetyMonitorBase("CommunicationMonitor") {}

    void configure(const Config& config) { _config = config; }
    void update() override;
    void update_manual_command_time(uint64_t timestamp);
    void update_vehicle_command_time(uint64_t timestamp);
    void update_module_status_time(uint64_t timestamp);

    const State& get_state() const { return _state; }
    float get_communication_quality() const { return _state.communication_quality; }
    bool is_communication_timeout() const {
        return _state.manual_control_timeout || _state.vehicle_command_timeout;
    }

private:
    Config _config;
    State _state;

    void check_timeouts();
    void calculate_quality();
};

} // namespace safety_manager

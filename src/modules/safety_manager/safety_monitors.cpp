#include "safety_monitors.hpp"
#include <mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

namespace safety_manager
{

void SpeedMonitor::update()
{
    calculate_acceleration();
    check_speed_limits();
    update_safe_speed();

    _state.last_update_time = hrt_absolute_time();
}

void SpeedMonitor::update_wheel_speed(float wheel_speed_rad_s)
{
    _state.current_speed_ms = wheel_speed_rad_s * _config.wheel_radius_m;
    _state.last_update_time = hrt_absolute_time();
}

void SpeedMonitor::calculate_acceleration()
{
    uint64_t current_time = hrt_absolute_time();

    if (_last_speed_time > 0) {
        float dt = (current_time - _last_speed_time) / 1e6f;
        if (dt > 0.001f) { // Avoid division by zero
            _state.acceleration_ms2 = (_state.current_speed_ms - _last_speed) / dt;
        }
    }

    _last_speed = _state.current_speed_ms;
    _last_speed_time = current_time;
}

void SpeedMonitor::check_speed_limits()
{
    if (_state.current_speed_ms > _config.max_speed_ms) {
        _state.speed_limit_exceeded = true;
        set_fault(FaultType::SPEED_LIMIT);
    } else {
        _state.speed_limit_exceeded = false;
        clear_fault(FaultType::SPEED_LIMIT);
    }

    // Check for hard braking
    _state.hard_braking_detected = (_state.acceleration_ms2 < -_config.emergency_decel_threshold);
}

void SpeedMonitor::update_safe_speed()
{
    // Base safe speed is the configured maximum
    float safety_factor = 1.0f;

    // Reduce safe speed based on other system conditions
    // This would typically be updated by other monitors or external conditions
    _state.max_safe_speed_ms = _config.max_speed_ms * safety_factor;

    // Update health score based on speed compliance
    float speed_factor = 1.0f - math::constrain(_state.current_speed_ms / _config.max_speed_ms, 0.0f, 1.0f);
    update_health_score(speed_factor);
}

void SteeringMonitor::update()
{
    calculate_steering_rate();
    check_angle_limits();
    check_rate_limits();
    check_steering_error();

    _state.last_update_time = hrt_absolute_time();
}

void SteeringMonitor::update_current_angle(float angle_rad)
{
    _state.current_angle_rad = angle_rad;
    _state.last_update_time = hrt_absolute_time();
}

void SteeringMonitor::update_commanded_angle(float angle_rad)
{
    _state.commanded_angle_rad = angle_rad;
}

void SteeringMonitor::calculate_steering_rate()
{
    uint64_t current_time = hrt_absolute_time();

    if (_last_angle_time > 0) {
        float dt = (current_time - _last_angle_time) / 1e6f;
        if (dt > 0.001f) {
            _state.steering_rate_rad_s = (_state.current_angle_rad - _last_angle) / dt;
        }
    }

    _last_angle = _state.current_angle_rad;
    _last_angle_time = current_time;
}

void SteeringMonitor::check_angle_limits()
{
    if (fabsf(_state.current_angle_rad) > _config.max_angle_rad) {
        _state.angle_limit_exceeded = true;
        set_fault(FaultType::STEERING_LIMIT);
    } else {
        _state.angle_limit_exceeded = false;
        clear_fault(FaultType::STEERING_LIMIT);
    }
}

void SteeringMonitor::check_rate_limits()
{
    _state.rate_limit_exceeded = (fabsf(_state.steering_rate_rad_s) > _config.max_rate_rad_s);
}

void SteeringMonitor::check_steering_error()
{
    float error = fabsf(_state.current_angle_rad - _state.commanded_angle_rad);
    _state.steering_error_detected = (error > _config.error_threshold_rad);

    // Update health score based on steering error
    float error_factor = 1.0f - math::constrain(error / (_config.error_threshold_rad * 2.0f), 0.0f, 1.0f);
    update_health_score(error_factor);
}

void StabilityMonitor::update()
{
    check_stability_limits();
    calculate_stability_margin();
    assess_rollover_risk();

    _state.last_update_time = hrt_absolute_time();
}

void StabilityMonitor::update_imu_data(float accel_x, float accel_y, float accel_z)
{
    calculate_attitude_from_accel(accel_x, accel_y, accel_z);
    _state.lateral_acceleration_ms2 = accel_y;
    _state.longitudinal_acceleration_ms2 = accel_x;
    _state.last_update_time = hrt_absolute_time();
}

void StabilityMonitor::update_gyro_data(float gyro_x, float gyro_y, float gyro_z)
{
    _state.roll_rate_rad_s = gyro_x;
    _state.pitch_rate_rad_s = gyro_y;
}

void StabilityMonitor::calculate_attitude_from_accel(float accel_x, float accel_y, float accel_z)
{
    // Calculate roll and pitch from accelerometer data
    _state.roll_angle_rad = atan2f(accel_y, accel_z);
    _state.pitch_angle_rad = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));
}

void StabilityMonitor::check_stability_limits()
{
    bool roll_exceeded = fabsf(_state.roll_angle_rad) > _config.max_roll_angle_rad;
    bool pitch_exceeded = fabsf(_state.pitch_angle_rad) > _config.max_pitch_angle_rad;

    if (roll_exceeded || pitch_exceeded) {
        _state.stability_warning = true;
        set_fault(FaultType::STABILITY);
    } else {
        _state.stability_warning = false;
        clear_fault(FaultType::STABILITY);
    }
}

void StabilityMonitor::calculate_stability_margin()
{
    float roll_margin = 1.0f - fabsf(_state.roll_angle_rad) / _config.max_roll_angle_rad;
    float pitch_margin = 1.0f - fabsf(_state.pitch_angle_rad) / _config.max_pitch_angle_rad;

    _state.stability_margin = math::min(math::max(roll_margin, 0.0f), math::max(pitch_margin, 0.0f));

    // Update health score
    update_health_score(_state.stability_margin);
}

void StabilityMonitor::assess_rollover_risk()
{
    _state.rollover_risk = (_state.stability_margin < _config.rollover_threshold);

    // Detect critical tip-over conditions
    _state.tip_over_detected = (fabsf(_state.roll_angle_rad) > _config.max_roll_angle_rad * 0.9f) ||
                               (fabsf(_state.pitch_angle_rad) > _config.max_pitch_angle_rad * 0.9f);
}

void LoadMonitor::update()
{
    check_load_limits();
    detect_load_shift();

    _state.last_update_time = hrt_absolute_time();
}

void LoadMonitor::update_load_data(float payload_kg, float cg_offset_m)
{
    _state.payload_mass_kg = payload_kg;
    _state.center_of_gravity_offset_m = cg_offset_m;
    _state.last_update_time = hrt_absolute_time();
}

void LoadMonitor::check_load_limits()
{
    if (_state.payload_mass_kg > _config.max_payload_kg) {
        _state.overload_detected = true;
        set_fault(FaultType::LOAD);
    } else {
        _state.overload_detected = false;
        clear_fault(FaultType::LOAD);
    }

    // Check center of gravity offset
    _state.unsafe_load_distribution =
        (fabsf(_state.center_of_gravity_offset_m) > _config.max_cg_offset_m);

    // Update health score based on load conditions
    float load_factor = 1.0f - math::constrain(_state.payload_mass_kg / _config.max_payload_kg, 0.0f, 1.0f);
    float cg_factor = 1.0f - math::constrain(fabsf(_state.center_of_gravity_offset_m) / _config.max_cg_offset_m, 0.0f, 1.0f);

    update_health_score((load_factor + cg_factor) / 2.0f);
}

void LoadMonitor::detect_load_shift()
{
    float cg_change = fabsf(_state.center_of_gravity_offset_m - _last_cg_offset);
    _state.load_shift_detected = (cg_change > _config.load_shift_threshold_m);
    _last_cg_offset = _state.center_of_gravity_offset_m;
}

void CommunicationMonitor::update()
{
    check_timeouts();
    calculate_quality();

    update_health_score(_state.communication_quality);
}

void CommunicationMonitor::update_manual_command_time(uint64_t timestamp)
{
    _state.last_manual_command_time = timestamp;
}

void CommunicationMonitor::update_vehicle_command_time(uint64_t timestamp)
{
    _state.last_vehicle_command_time = timestamp;
}

void CommunicationMonitor::update_module_status_time(uint64_t timestamp)
{
    _state.last_module_status_time = timestamp;
}

void CommunicationMonitor::check_timeouts()
{
    uint64_t current_time = hrt_absolute_time();

    // Check manual control timeout
    if (is_timeout(_state.last_manual_command_time, _config.timeout_us)) {
        if (!_state.manual_control_timeout) {
            _state.manual_control_timeout = true;
            _state.communication_failures++;
            set_fault(FaultType::COMMUNICATION);
        }
    } else {
        _state.manual_control_timeout = false;
    }

    // Check vehicle command timeout
    _state.vehicle_command_timeout = is_timeout(_state.last_vehicle_command_time, _config.timeout_us);

    // Check module status timeout
    _state.module_status_timeout = is_timeout(_state.last_module_status_time, _config.timeout_us);

    // Clear communication fault if no critical timeouts
    if (!_state.manual_control_timeout) {
        clear_fault(FaultType::COMMUNICATION);
    }
}

void CommunicationMonitor::calculate_quality()
{
    uint32_t timeout_count = 0;
    if (_state.manual_control_timeout) timeout_count++;
    if (_state.vehicle_command_timeout) timeout_count++;
    if (_state.module_status_timeout) timeout_count++;

    _state.communication_quality = 1.0f - (static_cast<float>(timeout_count) / 3.0f);
}

} // namespace safety_manager

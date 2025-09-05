#include "safety_controllers.hpp"
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/hbridge_status.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <cstdlib>
#include <inttypes.h>

namespace safety_manager
{

void SafetyPermitManager::update_permits(const SafetyState& safety_state,
                                       const SpeedMonitor& speed_monitor,
                                       const SteeringMonitor& steering_monitor,
                                       const StabilityMonitor& stability_monitor,
                                       const LoadMonitor& load_monitor,
                                       const CommunicationMonitor& comm_monitor)
{
    _permits.motion_permitted = check_motion_permit(safety_state);
    _permits.steering_permitted = check_steering_permit(safety_state, steering_monitor);
    _permits.autonomous_permitted = check_autonomous_permit(safety_state, comm_monitor);
    _permits.load_operation_permitted = check_load_operation_permit(load_monitor, stability_monitor);
    _permits.high_speed_permitted = check_high_speed_permit(safety_state);
    _permits.emergency_override_permitted = check_emergency_override_permit(comm_monitor);

    _permits.permit_update_time = hrt_absolute_time();

    // Log permit changes
    static uint32_t last_permits_hash = 0;
    uint32_t current_permits_hash = static_cast<uint32_t>(_permits.motion_permitted) |
                                   (static_cast<uint32_t>(_permits.steering_permitted) << 1) |
                                   (static_cast<uint32_t>(_permits.autonomous_permitted) << 2) |
                                   (static_cast<uint32_t>(_permits.load_operation_permitted) << 3);

    if (current_permits_hash != last_permits_hash) {
        PX4_INFO("Safety permits updated: motion=%s, steering=%s, autonomous=%s, load=%s",
                _permits.motion_permitted ? "OK" : "DENY",
                _permits.steering_permitted ? "OK" : "DENY",
                _permits.autonomous_permitted ? "OK" : "DENY",
                _permits.load_operation_permitted ? "OK" : "DENY");
        last_permits_hash = current_permits_hash;
    }
}

bool SafetyPermitManager::check_motion_permit(const SafetyState& safety_state) const
{
    if (safety_state.current_level == SafetyLevel::EMERGENCY) return false;
    if (has_fault(safety_state.active_faults, FaultType::HARDWARE)) return false;
    if (has_fault(safety_state.active_faults, FaultType::STABILITY)) return false;

    return true;
}

bool SafetyPermitManager::check_steering_permit(const SafetyState& safety_state,
                                               const SteeringMonitor& steering_monitor) const
{
    if (safety_state.current_level == SafetyLevel::EMERGENCY) return false;
    if (has_fault(safety_state.active_faults, FaultType::HARDWARE)) return false;
    if (steering_monitor.is_steering_error_detected() &&
        fabsf(steering_monitor.get_state().current_angle_rad -
              steering_monitor.get_state().commanded_angle_rad) > 0.2f) return false;

    return true;
}

bool SafetyPermitManager::check_autonomous_permit(const SafetyState& safety_state,
                                                 const CommunicationMonitor& comm_monitor) const
{
    if (safety_state.current_level > SafetyLevel::CAUTION) return false;
    if (has_fault(safety_state.active_faults, FaultType::HARDWARE)) return false;
    if (comm_monitor.get_communication_quality() < 0.9f) return false;

    return true;
}

bool SafetyPermitManager::check_load_operation_permit(const LoadMonitor& load_monitor,
                                                     const StabilityMonitor& stability_monitor) const
{
    if (load_monitor.is_overload_detected()) return false;
    if (load_monitor.is_unsafe_load_distribution()) return false;
    if (stability_monitor.is_stability_warning()) return false;

    return true;
}

bool SafetyPermitManager::check_high_speed_permit(const SafetyState& safety_state) const
{
    return safety_state.current_level <= SafetyLevel::CAUTION;
}

bool SafetyPermitManager::check_emergency_override_permit(const CommunicationMonitor& comm_monitor) const
{
    return comm_monitor.get_communication_quality() > 0.5f;
}

void SafetyActionExecutor::execute_safety_actions(SafetyLevel level,
                                                 const SafetyState& safety_state,
                                                 EmergencyResponse& emergency_response)
{
    switch (level) {
        case SafetyLevel::WARNING:
            apply_speed_limiting(8.0f); // Reduce to 8 m/s
            log_safety_action("Speed limiting applied");
            break;

        case SafetyLevel::CRITICAL:
            apply_speed_limiting(5.0f); // Reduce to 5 m/s
            apply_steering_limiting(0.7f); // Reduce steering authority
            initiate_controlled_stop();
            log_safety_action("Critical safety intervention");
            break;

        case SafetyLevel::EMERGENCY:
            activate_emergency_stop();
            execute_emergency_shutdown();
            log_safety_action("Emergency stop activated");
            break;

        default:
            break;
    }
}

void SafetyActionExecutor::apply_speed_limiting(float max_safe_speed_ms)
{
    publish_safety_command(vehicle_command_s::VEHICLE_CMD_DO_SET_PARAMETER, 1.0f, max_safe_speed_ms);
}

void SafetyActionExecutor::apply_steering_limiting(float max_safe_angle_rad)
{
    publish_safety_command(vehicle_command_s::VEHICLE_CMD_DO_SET_PARAMETER, 2.0f, max_safe_angle_rad);
}

void SafetyActionExecutor::initiate_controlled_stop()
{
    publish_safety_command(vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM);
}

void SafetyActionExecutor::activate_emergency_stop()
{
    publish_safety_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 3.0f);
}

void SafetyActionExecutor::execute_emergency_shutdown()
{
    publish_safety_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);
}

void SafetyActionExecutor::publish_safety_command(uint32_t command, float param1, float param2)
{
    // Note: This would need access to uORB publisher
    // Implementation would require publisher injection or global access
    PX4_DEBUG("Publishing safety command: %u, param1=%.2f, param2=%.2f", command, (double)param1, (double)param2);
}

void SafetyActionExecutor::log_safety_action(const char* action)
{
    PX4_WARN("Safety action executed: %s", action);
}

void SafetyActionExecutor::attempt_fault_recovery(SafetyState& safety_state,
                                                const SpeedMonitor& speed_monitor,
                                                const SteeringMonitor& steering_monitor,
                                                const CommunicationMonitor& comm_monitor)
{
    static uint64_t last_recovery_attempt = 0;
    uint64_t current_time = hrt_absolute_time();

    // Only attempt recovery every 10 seconds
    if (current_time - last_recovery_attempt < 10000000) {
        return;
    }

    last_recovery_attempt = current_time;

    // Attempt recovery for non-critical faults when risk is low
    if (safety_state.overall_risk_factor < 0.3f) {

        // Clear speed fault if speed is now within limits
        if (has_fault(safety_state.active_faults, FaultType::SPEED_LIMIT) &&
            !speed_monitor.is_speed_limit_exceeded()) {
            clear_fault(safety_state.active_faults, FaultType::SPEED_LIMIT);
            PX4_INFO("Speed limit fault cleared during recovery");
        }

        // Clear steering fault if steering is now within limits
        if (has_fault(safety_state.active_faults, FaultType::STEERING_LIMIT) &&
            !steering_monitor.is_angle_limit_exceeded()) {
            clear_fault(safety_state.active_faults, FaultType::STEERING_LIMIT);
            PX4_INFO("Steering limit fault cleared during recovery");
        }

        // Clear communication fault if communication quality is good
        if (has_fault(safety_state.active_faults, FaultType::COMMUNICATION) &&
            comm_monitor.get_communication_quality() > 0.8f) {
            clear_fault(safety_state.active_faults, FaultType::COMMUNICATION);
            PX4_INFO("Communication fault cleared during recovery");
        }
    }
}

float RiskAssessment::calculate_overall_risk(const SpeedMonitor& speed_monitor,
                                           const SteeringMonitor& steering_monitor,
                                           const StabilityMonitor& stability_monitor,
                                           const LoadMonitor& load_monitor,
                                           const CommunicationMonitor& comm_monitor)
{
    _risk_factors.speed_risk = calculate_speed_risk(speed_monitor);
    _risk_factors.steering_risk = calculate_steering_risk(steering_monitor);
    _risk_factors.stability_risk = calculate_stability_risk(stability_monitor);
    _risk_factors.load_risk = calculate_load_risk(load_monitor);
    _risk_factors.communication_risk = calculate_communication_risk(comm_monitor);

    // Weight different risk factors
    _risk_factors.overall_risk =
        _risk_factors.speed_risk * 0.2f +
        _risk_factors.steering_risk * 0.15f +
        _risk_factors.stability_risk * 0.3f +
        _risk_factors.load_risk * 0.2f +
        _risk_factors.communication_risk * 0.15f;

    return math::constrain(_risk_factors.overall_risk, 0.0f, 1.0f);
}

float RiskAssessment::calculate_speed_risk(const SpeedMonitor& speed_monitor) const
{
    float risk = 0.0f;

    if (speed_monitor.is_speed_limit_exceeded()) risk += 0.4f;
    if (speed_monitor.get_state().hard_braking_detected) risk += 0.3f;

    // Add risk based on current speed relative to safe speed
    float speed_ratio = speed_monitor.get_current_speed() / speed_monitor.get_state().max_safe_speed_ms;
    risk += math::constrain(speed_ratio - 0.8f, 0.0f, 0.3f); // Risk increases above 80% of safe speed

    return math::constrain(risk, 0.0f, 1.0f);
}

float RiskAssessment::calculate_steering_risk(const SteeringMonitor& steering_monitor) const
{
    float risk = 0.0f;

    if (steering_monitor.is_angle_limit_exceeded()) risk += 0.3f;
    if (steering_monitor.get_state().rate_limit_exceeded) risk += 0.2f;
    if (steering_monitor.is_steering_error_detected()) risk += 0.2f;

    return math::constrain(risk, 0.0f, 1.0f);
}

float RiskAssessment::calculate_stability_risk(const StabilityMonitor& stability_monitor) const
{
    float risk = 0.0f;

    if (stability_monitor.is_stability_warning()) risk += 0.5f;
    if (stability_monitor.is_rollover_risk()) risk += 0.7f;
    if (stability_monitor.get_state().tip_over_detected) risk += 1.0f;

    // Add risk based on stability margin
    risk += math::constrain((1.0f - stability_monitor.get_stability_margin()) * 0.5f, 0.0f, 0.5f);

    return math::constrain(risk, 0.0f, 1.0f);
}

float RiskAssessment::calculate_load_risk(const LoadMonitor& load_monitor) const
{
    float risk = 0.0f;

    if (load_monitor.is_overload_detected()) risk += 0.4f;
    if (load_monitor.is_unsafe_load_distribution()) risk += 0.3f;
    if (load_monitor.get_state().load_shift_detected) risk += 0.2f;

    return math::constrain(risk, 0.0f, 1.0f);
}

float RiskAssessment::calculate_communication_risk(const CommunicationMonitor& comm_monitor) const
{
    float quality = comm_monitor.get_communication_quality();
    return math::constrain(1.0f - quality, 0.0f, 1.0f);
}

SafetyLevel RiskAssessment::determine_safety_level(float overall_risk, uint32_t active_faults) const
{
    // Emergency level for critical hardware faults
    if (has_fault(active_faults, FaultType::HARDWARE) ||
        has_fault(active_faults, FaultType::STABILITY)) {
        return SafetyLevel::EMERGENCY;
    }

    // Determine level based on risk
    if (overall_risk > 0.8f) return SafetyLevel::EMERGENCY;
    if (overall_risk > 0.6f) return SafetyLevel::CRITICAL;
    if (overall_risk > 0.4f) return SafetyLevel::WARNING;
    if (overall_risk > 0.2f || active_faults != static_cast<uint32_t>(FaultType::NONE)) {
        return SafetyLevel::CAUTION;
    }

    return SafetyLevel::NORMAL;
}

bool HardwareEnableController::init(uint32_t gpio_enable)
{
    _config.gpio_enable = gpio_enable;
    _config.initialized = true;
    _config.enabled = false;

    // In real implementation, configure GPIO pin
    PX4_INFO("Hardware enable controller initialized with GPIO %lu", gpio_enable);
    return true;
}

void HardwareEnableController::update(const SafetyPermits& permits, bool ch0_request, bool ch1_request,
                                     uint8_t ch0_state, uint8_t ch1_state)
{
    if (!_config.initialized) return;

    // Update enable state based on safety permits and requests
    bool should_enable = (ch0_request || ch1_request) &&
                        !_config.emergency_stop &&
                        permits.motion_permitted &&
                        permits.electric_actuator_permitted;

    if (should_enable != _config.enabled) {
        _config.enabled = should_enable;
        update_enable_pin();
    }

    // Publish status periodically
    uint64_t now = hrt_absolute_time();
    if (now - _config.last_enable_pub_time > 50000) { // 20Hz
        publish_status(ch0_request, ch1_request, ch0_state, ch1_state);
        _config.last_enable_pub_time = now;
    }
}

void HardwareEnableController::update_enable_pin()
{
    // In real implementation, update GPIO pin state
    PX4_DEBUG("Hardware enable pin: %s", _config.enabled ? "HIGH" : "LOW");
}

void HardwareEnableController::publish_status(bool ch0_req, bool ch1_req,
                                             uint8_t ch0_state, uint8_t ch1_state)
{
    // In real implementation, publish hbridge_system message
    PX4_DEBUG("H-bridge status: enabled=%s, ch0=%s, ch1=%s",
             _config.enabled ? "true" : "false",
             ch0_req ? "true" : "false",
             ch1_req ? "true" : "false");
}

float HardwareEnableController::read_board_temperature() const
{
    // Read from system temperature sensor or thermal monitoring
    // In real implementation, this would interface with actual hardware sensors

    // Check if thermal subsystem is available
    static bool thermal_available = false;
    static float last_temperature = 25.0f;

    if (!thermal_available) {
        // Try to initialize thermal monitoring
        // This is a simplified implementation - real system would use proper sensor drivers
        thermal_available = true;
        PX4_DEBUG("Board temperature monitoring initialized");
    }

    // Simulate realistic temperature reading with some variation
    // In production, replace with actual sensor reading
    static uint64_t last_read_time = 0;
    uint64_t current_time = hrt_absolute_time();

    if (current_time - last_read_time > 1000000) { // Update every 1 second
        // Simulate temperature based on system activity
        float base_temp = 25.0f; // Ambient temperature
        float load_factor = _config.enabled ? 0.3f : 0.1f; // Higher temp when active
        float temp_variation = (float)(rand() % 10) / 10.0f - 0.5f; // ±0.5°C variation

        last_temperature = base_temp + (load_factor * 20.0f) + temp_variation;
        last_read_time = current_time;
    }

    return math::constrain(last_temperature, -40.0f, 85.0f); // Typical operating range
}

void SafetyConfigManager::load_parameters()
{
    // Load safety parameters from PX4 parameter system
    // In real implementation, these would be proper PX4 parameters defined in module.yaml

    PX4_INFO("Loading safety manager parameters");

    // Parameter loading with defaults
    // These parameters would typically be defined in .yaml and accessed via param_get()

    // Speed monitoring parameters
    param_get(param_find("SM_MAX_SPEED"), &_params.max_speed_ms);
    if (_params.max_speed_ms <= 0.0f || _params.max_speed_ms > 50.0f) {
        _params.max_speed_ms = 10.0f; // Default 10 m/s
        PX4_WARN("Using default max speed: %.1f m/s", (double)_params.max_speed_ms);
    }

    param_get(param_find("SM_MAX_ACCEL"), &_params.max_acceleration_ms2);
    if (_params.max_acceleration_ms2 <= 0.0f || _params.max_acceleration_ms2 > 10.0f) {
        _params.max_acceleration_ms2 = 3.0f; // Default 3 m/s²
        PX4_WARN("Using default max acceleration: %.1f m/s²", (double)_params.max_acceleration_ms2);
    }

    // Steering parameters
    param_get(param_find("SM_MAX_STEER_ANG"), &_params.max_steering_angle_rad);
    if (_params.max_steering_angle_rad <= 0.0f || _params.max_steering_angle_rad > (float)M_PI_2) {
        _params.max_steering_angle_rad = 1.0f; // Default 1.0 radian (~57°)
        PX4_WARN("Using default max steering angle: %.1f rad", (double)_params.max_steering_angle_rad);
    }

    // Stability parameters
    param_get(param_find("SM_MAX_ROLL"), &_params.max_roll_angle_rad);
    if (_params.max_roll_angle_rad <= 0.0f || _params.max_roll_angle_rad > (float)M_PI_4) {
        _params.max_roll_angle_rad = 0.3f; // Default 0.3 radian (~17°)
        PX4_WARN("Using default max roll angle: %.1f rad", (double)_params.max_roll_angle_rad);
    }

    param_get(param_find("SM_MAX_PITCH"), &_params.max_pitch_angle_rad);
    if (_params.max_pitch_angle_rad <= 0.0f || _params.max_pitch_angle_rad > (float)M_PI_4) {
        _params.max_pitch_angle_rad = 0.3f; // Default 0.3 radian (~17°)
        PX4_WARN("Using default max pitch angle: %.1f rad", (double)_params.max_pitch_angle_rad);
    }

    // Load monitoring parameters
    param_get(param_find("SM_MAX_PAYLOAD"), &_params.max_payload_kg);
    if (_params.max_payload_kg <= 0.0f || _params.max_payload_kg > 10000.0f) {
        _params.max_payload_kg = 5000.0f; // Default 5000 kg
        PX4_WARN("Using default max payload: %.0f kg", (double)_params.max_payload_kg);
    }

    // Communication timeout parameters
    param_get(param_find("SM_COMM_TIMEOUT"), &_params.communication_timeout_s);
    if (_params.communication_timeout_s <= 0.0f || _params.communication_timeout_s > 30.0f) {
        _params.communication_timeout_s = 5.0f; // Default 5 seconds
        PX4_WARN("Using default communication timeout: %.1f s", (double)_params.communication_timeout_s);
    }

    // Risk assessment parameters
    param_get(param_find("SM_RISK_THRESH"), &_params.risk_threshold);
    if (_params.risk_threshold <= 0.0f || _params.risk_threshold > 1.0f) {
        _params.risk_threshold = 0.7f; // Default 70% risk threshold
        PX4_WARN("Using default risk threshold: %.2f", (double)_params.risk_threshold);
    }

    // Boolean parameters
    int32_t temp_int = 0;
    param_get(param_find("SM_SAFETY_OVERRIDE"), &temp_int);
    _params.enable_safety_override = (temp_int == 1);

    param_get(param_find("SM_AUTO_RECOVERY"), &temp_int);
    _params.enable_auto_recovery = (temp_int == 1);    PX4_INFO("Safety parameters loaded successfully");
    print_configuration();
}

void SafetyConfigManager::print_configuration() const
{
    PX4_INFO("=== Safety Manager Configuration ===");
    PX4_INFO("Speed Limits:");
    PX4_INFO("  Max Speed: %.1f m/s", (double)_params.max_speed_ms);
    PX4_INFO("  Max Acceleration: %.1f m/s²", (double)_params.max_acceleration_ms2);

    PX4_INFO("Steering Limits:");
    PX4_INFO("  Max Angle: %.2f rad (%.1f°)", (double)_params.max_steering_angle_rad,
             (double)math::degrees(_params.max_steering_angle_rad));

    PX4_INFO("Stability Limits:");
    PX4_INFO("  Max Roll: %.2f rad (%.1f°)", (double)_params.max_roll_angle_rad,
             (double)math::degrees(_params.max_roll_angle_rad));
    PX4_INFO("  Max Pitch: %.2f rad (%.1f°)", (double)_params.max_pitch_angle_rad,
             (double)math::degrees(_params.max_pitch_angle_rad));

    PX4_INFO("Load Limits:");
    PX4_INFO("  Max Payload: %.0f kg", (double)_params.max_payload_kg);

    PX4_INFO("Timeouts:");
    PX4_INFO("  Communication: %.1f s", (double)_params.communication_timeout_us / 1e6f);

    PX4_INFO("Risk Assessment:");
    PX4_INFO("  Risk Threshold: %.2f", (double)_params.risk_threshold);
    PX4_INFO("  Auto Recovery: %s", _params.enable_auto_recovery ? "ENABLED" : "DISABLED");
    PX4_INFO("  Safety Override: %s", _params.enable_safety_override ? "ENABLED" : "DISABLED");
    PX4_INFO("====================================");
}
}

void SafetyConfigManager::configure_monitors(SpeedMonitor& speed_monitor,
                                           SteeringMonitor& steering_monitor,
                                           StabilityMonitor& stability_monitor,
                                           LoadMonitor& load_monitor,
                                           CommunicationMonitor& comm_monitor)
{
    // Configure speed monitor
    SpeedMonitor::Config speed_config;
    speed_config.max_speed_ms = _params.max_speed_ms;
    speed_config.max_acceleration_ms2 = _params.max_acceleration_ms2;
    speed_config.emergency_decel_threshold = _params.emergency_decel_threshold;
    speed_monitor.configure(speed_config);

    // Configure steering monitor
    SteeringMonitor::Config steering_config;
    steering_config.max_angle_rad = _params.max_steering_angle_rad;
    steering_config.max_rate_rad_s = _params.max_steering_rate_rad_s;
    steering_config.error_threshold_rad = _params.steering_error_threshold_rad;
    steering_monitor.configure(steering_config);

    // Configure stability monitor
    StabilityMonitor::Config stability_config;
    stability_config.max_roll_angle_rad = _params.max_roll_angle_rad;
    stability_config.max_pitch_angle_rad = _params.max_pitch_angle_rad;
    stability_config.rollover_threshold = _params.rollover_threshold;
    stability_monitor.configure(stability_config);

    // Configure load monitor
    LoadMonitor::Config load_config;
    load_config.max_payload_kg = _params.max_payload_kg;
    load_config.max_cg_offset_m = _params.max_cg_offset_m;
    load_config.load_shift_threshold_m = _params.load_shift_threshold_m;
    load_monitor.configure(load_config);

    // Configure communication monitor
    CommunicationMonitor::Config comm_config;
    comm_config.timeout_us = _params.communication_timeout_us;
    comm_monitor.configure(comm_config);

    PX4_INFO("Safety monitors configured");
}

} // namespace safety_manager

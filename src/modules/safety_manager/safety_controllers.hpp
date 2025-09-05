#pragma once

#include "safety_types.hpp"
#include "safety_monitors.hpp"
#include <px4_platform_common/log.h>

namespace safety_manager
{

/**
 * @brief Safety permit manager
 *
 * Handles all safety permits based on monitor states and system conditions
 */
class SafetyPermitManager
{
public:
    SafetyPermitManager() = default;
    ~SafetyPermitManager() = default;

    // Update all safety permits
    void update_permits(const SafetyState& safety_state,
                       const SpeedMonitor& speed_monitor,
                       const SteeringMonitor& steering_monitor,
                       const StabilityMonitor& stability_monitor,
                       const LoadMonitor& load_monitor,
                       const CommunicationMonitor& comm_monitor);

    // Individual permit checks
    bool check_motion_permit(const SafetyState& safety_state) const;
    bool check_steering_permit(const SafetyState& safety_state,
                              const SteeringMonitor& steering_monitor) const;
    bool check_autonomous_permit(const SafetyState& safety_state,
                               const CommunicationMonitor& comm_monitor) const;
    bool check_load_operation_permit(const LoadMonitor& load_monitor,
                                   const StabilityMonitor& stability_monitor) const;
    bool check_high_speed_permit(const SafetyState& safety_state) const;
    bool check_emergency_override_permit(const CommunicationMonitor& comm_monitor) const;

    // Get current permits
    const SafetyPermits& get_permits() const { return _permits; }

private:
    SafetyPermits _permits{};
};

/**
 * @brief Safety action executor
 *
 * Executes safety actions and fail-safe behaviors
 */
class SafetyActionExecutor
{
public:
    SafetyActionExecutor() = default;
    ~SafetyActionExecutor() = default;

    // Execute safety actions based on safety level
    void execute_safety_actions(SafetyLevel level,
                               const SafetyState& safety_state,
                               EmergencyResponse& emergency_response);

    // Individual safety actions
    void apply_speed_limiting(float max_safe_speed_ms);
    void apply_steering_limiting(float max_safe_angle_rad);
    void initiate_controlled_stop();
    void activate_emergency_stop();
    void execute_emergency_shutdown();

    // Recovery actions
    void attempt_fault_recovery(SafetyState& safety_state,
                               const SpeedMonitor& speed_monitor,
                               const SteeringMonitor& steering_monitor,
                               const CommunicationMonitor& comm_monitor);

private:
    void publish_safety_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f);
    void log_safety_action(const char* action);
};

/**
 * @brief Risk assessment calculator
 *
 * Calculates overall system risk based on various factors
 */
class RiskAssessment
{
public:
    struct RiskFactors {
        float speed_risk{0.0f};
        float steering_risk{0.0f};
        float stability_risk{0.0f};
        float load_risk{0.0f};
        float communication_risk{0.0f};
        float hardware_risk{0.0f};
        float sensor_risk{0.0f};
        float overall_risk{0.0f};
    };

    RiskAssessment() = default;
    ~RiskAssessment() = default;

    // Calculate overall risk factor
    float calculate_overall_risk(const SpeedMonitor& speed_monitor,
                               const SteeringMonitor& steering_monitor,
                               const StabilityMonitor& stability_monitor,
                               const LoadMonitor& load_monitor,
                               const CommunicationMonitor& comm_monitor);

    // Get detailed risk factors
    const RiskFactors& get_risk_factors() const { return _risk_factors; }

    // Determine safety level from risk
    SafetyLevel determine_safety_level(float overall_risk, uint32_t active_faults) const;

private:
    RiskFactors _risk_factors{};

    float calculate_speed_risk(const SpeedMonitor& speed_monitor) const;
    float calculate_steering_risk(const SteeringMonitor& steering_monitor) const;
    float calculate_stability_risk(const StabilityMonitor& stability_monitor) const;
    float calculate_load_risk(const LoadMonitor& load_monitor) const;
    float calculate_communication_risk(const CommunicationMonitor& comm_monitor) const;
};

/**
 * @brief Hardware enable controller
 *
 * Manages hardware enable pins and limit sensor integration
 */
class HardwareEnableController
{
public:
    HardwareEnableController() = default;
    ~HardwareEnableController() = default;

    bool init(uint32_t gpio_enable);
    void update(const SafetyPermits& permits, bool ch0_request, bool ch1_request,
               uint8_t ch0_state, uint8_t ch1_state);

    // Limit sensor configuration
    void set_limit_config(uint8_t channel, uint8_t min_limit_instance, uint8_t max_limit_instance,
                         bool allow_into_min = false, bool allow_into_max = false);

    // Motion permission checks
    bool check_motion_allowed_for_channel(uint8_t channel, float command) const;
    bool is_limit_active(uint8_t instance) const;

    // State queries
    bool is_enabled() const { return _config.enabled; }
    bool is_emergency_stop_active() const { return _config.emergency_stop; }
    void clear_emergency_stop() { _config.emergency_stop = false; }

    // Temperature monitoring
    float read_board_temperature() const;

private:
    HardwareEnableConfig _config{};

    void update_enable_pin();
    void check_safety_override();
    void publish_status(bool ch0_req, bool ch1_req, uint8_t ch0_state, uint8_t ch1_state);
};

/**
 * @brief Safety system configuration manager
 *
 * Handles parameter loading and configuration management
 */
class SafetyConfigManager
{
public:
    struct SafetyParameters {
        // Speed parameters
        float max_speed_ms{10.0f};
        float max_acceleration_ms2{3.0f};
        float emergency_decel_threshold{5.0f};

        // Steering parameters
        float max_steering_angle_rad{1.0f};
        float max_steering_rate_rad_s{0.5f};
        float steering_error_threshold_rad{0.1f};

        // Stability parameters
        float max_roll_angle_rad{0.3f};
        float max_pitch_angle_rad{0.3f};
        float rollover_threshold{0.2f};

        // Load parameters
        float max_payload_kg{5000.0f};
        float max_cg_offset_m{1.0f};
        float load_shift_threshold_m{0.5f};

        // Communication parameters
        uint64_t communication_timeout_us{5000000};
        float communication_timeout_s{5.0f}; // Added for compatibility
        uint64_t sensor_timeout_us{1000000};

        // Risk assessment parameters
        float risk_threshold{0.7f};
        bool enable_auto_recovery{true};
        bool enable_safety_override{true};
    };

    SafetyConfigManager() = default;
    ~SafetyConfigManager() = default;

    void load_parameters();
    void configure_monitors(SpeedMonitor& speed_monitor,
                          SteeringMonitor& steering_monitor,
                          StabilityMonitor& stability_monitor,
                          LoadMonitor& load_monitor,
                          CommunicationMonitor& comm_monitor);
    void print_configuration() const;

    const SafetyParameters& get_parameters() const { return _params; }

private:
    SafetyParameters _params{};
};

} // namespace safety_manager

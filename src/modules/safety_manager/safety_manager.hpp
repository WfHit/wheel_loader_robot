#pragma once

#include "safety_types.hpp"
#include "safety_monitors.hpp"
#include "safety_controllers.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

// uORB message includes - System Status
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_command.h>

// uORB message includes - Chassis Control
#include <uORB/topics/drivetrain_setpoint.h>
#include <uORB/topics/steering_setpoint.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/traction_control.h>

// uORB message includes - Electric Actuator Systems
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/load_aware_torque.h>

// uORB message includes - Sensors and Navigation
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>

// uORB message includes - Environmental
#include <uORB/topics/wheel_loader_status.h>

// uORB message includes - Limit Sensors
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/hbridge_system.h>
#include <uORB/topics/input_rc.h>

using namespace time_literals;
using namespace safety_manager;

// Constants
#define MAX_LIMIT_SENSORS 8

/**
 * @brief Refactored Safety Manager for Wheel Loader
 *
 * Modular safety monitoring and fail-safe system that:
 * - Uses specialized monitor classes for different subsystems
 * - Implements centralized permit management
 * - Provides comprehensive risk assessment
 * - Maintains hardware enable control
 * - Supports configurable safety parameters
 * - Enables emergency response procedures
 *
 * Architecture:
 * - SafetyMonitors: Individual monitoring subsystems (speed, steering, etc.)
 * - SafetyPermitManager: Centralized permit decisions
 * - SafetyActionExecutor: Executes safety actions and fail-safes
 * - RiskAssessment: Calculates overall system risk
 * - HardwareEnableController: Manages hardware enable pins
 * - SafetyConfigManager: Handles parameter configuration
 */
class SafetyManager : public ModuleBase<SafetyManager>, public ModuleParams
{
public:
    SafetyManager();
    ~SafetyManager() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

private:
    void run() override;

    /**
     * Core safety management functions
     */
    void update_subscriptions();
    void update_monitors();
    void assess_safety_state();
    void update_permits_and_actions();
    void publish_safety_status();

    /**
     * Monitor update functions
     */
    void update_speed_monitor();
    void update_steering_monitor();
    void update_stability_monitor();
    void update_load_monitor();
    void update_communication_monitor();

    /**
     * Safety mode management
     */
    void handle_safety_mode_commands();
    void update_zeroing_mode();

    /**
     * Hardware enable management
     */
    void update_hardware_enable();

    // Safety subsystem components
    SpeedMonitor _speed_monitor;
    SteeringMonitor _steering_monitor;
    StabilityMonitor _stability_monitor;
    LoadMonitor _load_monitor;
    CommunicationMonitor _communication_monitor;

    // Safety management components
    SafetyPermitManager _permit_manager;
    SafetyActionExecutor _action_executor;
    RiskAssessment _risk_assessment;
    HardwareEnableController _hardware_enable_controller;
    SafetyConfigManager _config_manager;

    // Safety state
    SafetyState _safety_state{};
    SafetyPermits _safety_permits{};
    EmergencyResponse _emergency_response{};
    SafetyPerformance _safety_performance{};

    // uORB subscriptions - System status
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
    uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};

    // uORB subscriptions - Chassis Control
    uORB::Subscription _drivetrain_setpoint_sub{ORB_ID(drivetrain_setpoint)};
    uORB::Subscription _steering_setpoint_sub{ORB_ID(steering_setpoint)};
    uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};
    uORB::Subscription _traction_control_sub{ORB_ID(traction_control)};

    // uORB subscriptions - Electric Actuator Systems
    uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
    uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
    uORB::Subscription _wheel_loader_status_sub{ORB_ID(wheel_loader_status)};

    // uORB subscriptions - Sensor data
    uORB::Subscription _imu_sub{ORB_ID(sensor_accel)};
    uORB::Subscription _gyro_sub{ORB_ID(sensor_gyro)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _load_aware_torque_sub{ORB_ID(load_aware_torque)};

    // uORB subscriptions - Limit sensors (up to 8 instances)
    uORB::Subscription _limit_sensor_sub[MAX_LIMIT_SENSORS]{};

    // uORB subscriptions - Control commands
    uORB::Subscription _manual_control_sub{ORB_ID(input_rc)};
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

    // uORB publications - Safety outputs
    uORB::Publication<vehicle_command_s> _safety_command_pub{ORB_ID(vehicle_command)};
    uORB::Publication<hbridge_system_s> _hbridge_system_pub{ORB_ID(hbridge_system)};

    // Performance monitoring
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, "safety_manager: cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, "safety_manager: interval")};

    // Parameters - using PX4 parameter system placeholder (would need real parameter definitions)
    struct {
        float max_speed_ms{10.0f};
        float max_acceleration_ms2{3.0f};
        float emergency_decel_rate{5.0f};
        float max_steering_angle_rad{1.0f};
        float max_steering_rate_rads{0.5f};
        float max_roll_angle_rad{0.3f};
        float max_pitch_angle_rad{0.3f};
        float stability_margin_factor{0.2f};
        float max_payload_kg{5000.0f};
        float max_cg_offset_m{1.0f};
        float communication_timeout_s{5.0f};
        float sensor_timeout_s{1.0f};
        float risk_threshold{0.7f};
        bool enable_safety_override{true};
        bool enable_auto_recovery{true};
    } _params;
};

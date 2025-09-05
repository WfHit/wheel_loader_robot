#include "safety_manager.hpp"

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <commander/px4_custom_mode.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#define MODULE_NAME "safety_manager"

SafetyManager::SafetyManager() :
	ModuleBase(MODULE_NAME),
	ModuleParams(nullptr)
{
	// Initialize safety state
	_safety_state = {};
	_safety_permits = {};
	_emergency_response = {};
	_safety_performance = {};

	// Initialize limit sensor subscriptions
	for (int i = 0; i < MAX_LIMIT_SENSORS; i++) {
		_limit_sensor_sub[i] = uORB::Subscription(ORB_ID(limit_sensor), i);
	}
}

bool SafetyManager::init()
{
	if (!_speed_monitor.init()) {
		PX4_ERR("Failed to initialize speed monitor");
		return false;
	}

	if (!_steering_monitor.init()) {
		PX4_ERR("Failed to initialize steering monitor");
		return false;
	}

	if (!_stability_monitor.init()) {
		PX4_ERR("Failed to initialize stability monitor");
		return false;
	}

	if (!_load_monitor.init()) {
		PX4_ERR("Failed to initialize load monitor");
		return false;
	}

	if (!_communication_monitor.init()) {
		PX4_ERR("Failed to initialize communication monitor");
		return false;
	}

	if (!_permit_manager.init()) {
		PX4_ERR("Failed to initialize permit manager");
		return false;
	}

	if (!_action_executor.init()) {
		PX4_ERR("Failed to initialize action executor");
		return false;
	}

	if (!_risk_assessment.init()) {
		PX4_ERR("Failed to initialize risk assessment");
		return false;
	}

	if (!_hardware_enable_controller.init()) {
		PX4_ERR("Failed to initialize hardware enable controller");
		return false;
	}

	if (!_config_manager.init()) {
		PX4_ERR("Failed to initialize config manager");
		return false;
	}

	PX4_INFO("Safety Manager initialized successfully");
	return true;
}

void SafetyManager::run()
{
	if (!should_exit() && !init()) {
		PX4_ERR("Initialization failed");
		return;
	}

	// Main execution loop
	while (!should_exit()) {
		perf_begin(_loop_perf);
		perf_count(_loop_interval_perf);

		// Update all subscriptions
		update_subscriptions();

		// Update configuration parameters
		_config_manager.update_parameters(_params);

		// Update all monitors with latest data and parameters
		update_monitors();

		// Handle safety mode commands
		handle_safety_mode_commands();

		// Handle zeroing mode operations
		update_zeroing_mode();

		// Assess overall safety state
		assess_safety_state();

		// Update permits and execute safety actions based on assessment
		update_permits_and_actions();

		// Update hardware enable control
		update_hardware_enable();

		// Publish safety status
		publish_safety_status();

		perf_end(_loop_perf);

		// Run at 50Hz (20ms cycle time)
		px4_usleep(20000);
	}
}

void SafetyManager::update_subscriptions()
{
	// Update all uORB subscriptions
	_vehicle_status_sub.update();
	_actuator_armed_sub.update();
	_failsafe_flags_sub.update();
	_drivetrain_setpoint_sub.update();
	_steering_setpoint_sub.update();
	_steering_status_sub.update();
	_traction_control_sub.update();
	_boom_status_sub.update();
	_bucket_status_sub.update();
	// _wheel_loader_status_sub.update();
	_imu_sub.update();
	_gyro_sub.update();
	_vehicle_attitude_sub.update();
	_vehicle_local_position_sub.update();
	_manual_control_sub.update();
	_vehicle_command_sub.update();

	// Update limit sensor subscriptions
	for (int i = 0; i < MAX_LIMIT_SENSORS; i++) {
		_limit_sensor_sub[i].update();
	}
}

void SafetyManager::update_monitors()
{
	update_speed_monitor();
	update_steering_monitor();
	update_stability_monitor();
	update_load_monitor();
	update_communication_monitor();
}

void SafetyManager::update_speed_monitor()
{
	SpeedMonitorData data;
	data.timestamp = hrt_absolute_time();

	// Get vehicle position data
	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s pos = _vehicle_local_position_sub.get();
		data.current_speed_ms = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
		data.acceleration_ms2 = sqrtf(pos.ax * pos.ax + pos.ay * pos.ay);
	}

	// Get drivetrain setpoint
	if (_drivetrain_setpoint_sub.updated()) {
		drivetrain_setpoint_s setpoint = _drivetrain_setpoint_sub.get();
		data.commanded_speed_ms = setpoint.speed;
	}

	// Update thresholds from parameters
	data.max_speed_threshold = _params.max_speed_ms;
	data.max_acceleration_threshold = _params.max_acceleration_ms2;
	data.emergency_decel_rate = _params.emergency_decel_rate;

	_speed_monitor.update(data);
}

void SafetyManager::update_steering_monitor()
{
	SteeringMonitorData data;
	data.timestamp = hrt_absolute_time();

	// Get steering status
	if (_steering_status_sub.updated()) {
		steering_status_s status = _steering_status_sub.get();
		data.current_angle_rad = status.steering_angle_rad;
		data.angular_velocity_rads = status.steering_rate_rads;
	}

	// Get steering setpoint
	if (_steering_setpoint_sub.updated()) {
		steering_setpoint_s setpoint = _steering_setpoint_sub.get();
		data.commanded_angle_rad = setpoint.steering_angle_rad;
	}

	// Update thresholds from parameters
	data.max_angle_threshold = _params.max_steering_angle_rad;
	data.max_rate_threshold = _params.max_steering_rate_rads;

	_steering_monitor.update(data);
}

void SafetyManager::update_stability_monitor()
{
	StabilityMonitorData data;
	data.timestamp = hrt_absolute_time();

	// Get vehicle attitude
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s attitude = _vehicle_attitude_sub.get();
		matrix::Eulerf euler(matrix::Quatf(attitude.q));
		data.roll_angle_rad = euler.phi();
		data.pitch_angle_rad = euler.theta();
	}

	// Get IMU data for rates
	if (_gyro_sub.updated()) {
		sensor_gyro_s gyro = _gyro_sub.get();
		data.roll_rate_rads = gyro.x;
		data.pitch_rate_rads = gyro.y;
	}

	// Get vehicle speed for stability analysis
	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s pos = _vehicle_local_position_sub.get();
		data.vehicle_speed_ms = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
	}

	// Update thresholds from parameters
	data.max_roll_threshold = _params.max_roll_angle_rad;
	data.max_pitch_threshold = _params.max_pitch_angle_rad;
	data.stability_margin = _params.stability_margin_factor;

	_stability_monitor.update(data);
}

void SafetyManager::update_load_monitor()
{
	LoadMonitorData data;
	data.timestamp = hrt_absolute_time();

	// Get boom status
	if (_boom_status_sub.updated()) {
		boom_status_s boom = _boom_status_sub.get();
		data.boom_angle_rad = boom.boom_angle_rad;
		data.boom_load_kg = boom.estimated_load_kg;
	}

	// Get bucket status
	if (_bucket_status_sub.updated()) {
		bucket_status_s bucket = _bucket_status_sub.get();
		data.bucket_angle_rad = bucket.bucket_angle_rad;
		data.bucket_load_kg = bucket.estimated_load_kg;
	}

	// Update thresholds from parameters
	data.max_payload_threshold = _params.max_payload_kg;
	data.max_cg_offset_threshold = _params.max_cg_offset_m;

	_load_monitor.update(data);
}

void SafetyManager::update_communication_monitor()
{
	CommunicationMonitorData data;
	data.timestamp = hrt_absolute_time();

	// Check manual control input
	if (_manual_control_sub.updated()) {
		input_rc_s manual = _manual_control_sub.get();
		data.last_manual_input = manual.timestamp;
		data.manual_control_valid = (manual.rc_lost == false);
	}

	// Check vehicle command updates
	if (_vehicle_command_sub.updated()) {
		vehicle_command_s cmd = _vehicle_command_sub.get();
		data.last_command_timestamp = cmd.timestamp;
	}

	// Update timeout thresholds from parameters
	data.communication_timeout_us = static_cast<uint64_t>(_params.communication_timeout_s * 1e6);
	data.sensor_timeout_us = static_cast<uint64_t>(_params.sensor_timeout_s * 1e6);

	_communication_monitor.update(data);
}

void SafetyManager::assess_safety_state()
{
	// Get monitor states
	_safety_state.speed_state = _speed_monitor.get_safety_state();
	_safety_state.steering_state = _steering_monitor.get_safety_state();
	_safety_state.stability_state = _stability_monitor.get_safety_state();
	_safety_state.load_state = _load_monitor.get_safety_state();
	_safety_state.communication_state = _communication_monitor.get_safety_state();

	// Update overall risk assessment
	RiskAssessmentData risk_data;
	risk_data.timestamp = hrt_absolute_time();
	risk_data.speed_risk = _speed_monitor.get_risk_level();
	risk_data.steering_risk = _steering_monitor.get_risk_level();
	risk_data.stability_risk = _stability_monitor.get_risk_level();
	risk_data.load_risk = _load_monitor.get_risk_level();
	risk_data.communication_risk = _communication_monitor.get_risk_level();
	risk_data.risk_threshold = _params.risk_threshold;

	_risk_assessment.update(risk_data);

	// Update overall safety level
	_safety_state.overall_safety_level = _risk_assessment.get_overall_safety_level();
	_safety_state.total_risk_score = _risk_assessment.get_total_risk_score();
	_safety_state.timestamp = hrt_absolute_time();
}

void SafetyManager::update_permits_and_actions()
{
	// Update safety permits based on monitor states
	SafetyPermitData permit_data;
	permit_data.timestamp = hrt_absolute_time();
	permit_data.speed_state = _safety_state.speed_state;
	permit_data.steering_state = _safety_state.steering_state;
	permit_data.stability_state = _safety_state.stability_state;
	permit_data.load_state = _safety_state.load_state;
	permit_data.communication_state = _safety_state.communication_state;
	permit_data.overall_safety_level = _safety_state.overall_safety_level;
	permit_data.enable_safety_override = _params.enable_safety_override;

	_permit_manager.update(permit_data);
	_safety_permits = _permit_manager.get_current_permits();

	// Execute safety actions based on permits and current state
	SafetyActionData action_data;
	action_data.timestamp = hrt_absolute_time();
	action_data.current_permits = _safety_permits;
	action_data.safety_state = _safety_state;
	action_data.enable_auto_recovery = _params.enable_auto_recovery;

	// Get vehicle armed state for action context
	if (_actuator_armed_sub.updated()) {
		actuator_armed_s armed = _actuator_armed_sub.get();
		action_data.system_armed = armed.armed;
		action_data.force_failsafe = armed.force_failsafe;
	}

	_action_executor.update(action_data);

	// Get emergency response if triggered
	_emergency_response = _action_executor.get_emergency_response();

	// Publish safety commands if needed
	if (_emergency_response.emergency_active) {
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = _emergency_response.command_id;
		cmd.param1 = _emergency_response.param1;
		cmd.param2 = _emergency_response.param2;
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd.source_system = 1;
		cmd.source_component = 1;
		cmd.from_external = false;

		_safety_command_pub.publish(cmd);
	}
}

void SafetyManager::handle_safety_mode_commands()
{
	if (_vehicle_command_sub.updated()) {
		vehicle_command_s cmd = _vehicle_command_sub.get();

		// Handle safety-specific commands
		switch (cmd.command) {
			case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
				// Safety system response to arm/disarm commands
				break;

			case vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE:
				// Handle guided mode safety requirements
				break;

			default:
				// Other commands handled by default action executor
				break;
		}
	}
}

void SafetyManager::update_zeroing_mode()
{
	// Handle actuator zeroing mode - special safety considerations
	// if (_wheel_loader_status_sub.updated()) {
	//	wheel_loader_status_s status = _wheel_loader_status_sub.get();

	//	if (status.zeroing_mode_active) {
	//		// Apply special safety constraints during zeroing
	//		// This typically involves:
	//		// - Reduced speed limits
	//		// - Enhanced monitoring of limit sensors
	//		// - Special fail-safe procedures for actuator protection

	//		// Update safety permits for zeroing mode
	//		_safety_permits.drive_permitted = false;  // No driving during zeroing
	//		_safety_permits.high_speed_permitted = false;
	//		_safety_permits.boom_operation_permitted = true;  // Allow boom movement for zeroing
	//		_safety_permits.bucket_operation_permitted = true;  // Allow bucket movement for zeroing
	//	}
	// }
}

void SafetyManager::update_hardware_enable()
{
	HardwareEnableData hw_data;
	hw_data.timestamp = hrt_absolute_time();
	hw_data.safety_permits = _safety_permits;
	hw_data.emergency_response = _emergency_response;
	hw_data.system_armed = false;

	// Get armed state
	if (_actuator_armed_sub.updated()) {
		actuator_armed_s armed = _actuator_armed_sub.get();
		hw_data.system_armed = armed.armed;
	}

	_hardware_enable_controller.update(hw_data);

	// Publish H-bridge system control
	hbridge_system_s hbridge{};
	hbridge.timestamp = hrt_absolute_time();
	hbridge.drive_enable = _hardware_enable_controller.is_drive_enabled();
	hbridge.boom_enable = _hardware_enable_controller.is_boom_enabled();
	hbridge.bucket_enable = _hardware_enable_controller.is_bucket_enabled();
	hbridge.steering_enable = _hardware_enable_controller.is_steering_enabled();
	hbridge.emergency_stop = _emergency_response.emergency_active;

	_hbridge_system_pub.publish(hbridge);
}

void SafetyManager::publish_safety_status()
{
	// Update safety performance metrics
	_safety_performance.cycle_time_us = static_cast<uint32_t>(perf_mean(_loop_perf));
	_safety_performance.max_cycle_time_us = static_cast<uint32_t>(perf_max(_loop_perf));
	_safety_performance.fault_count = _action_executor.get_fault_count();
	_safety_performance.recovery_count = _action_executor.get_recovery_count();

	// Publish comprehensive safety status to system
	// This would typically publish to a safety_status uORB topic for system-wide visibility
	// The topic would include all safety state information for monitoring and diagnostics

	PX4_DEBUG("Safety Status: Level=%d, Risk=%.2f, Faults=%u, Recoveries=%u",
			 static_cast<int>(_safety_state.overall_safety_level),
			 (double)_safety_state.total_risk_score,
			 _safety_performance.fault_count,
			 _safety_performance.recovery_count);
}

int SafetyManager::task_spawn(int argc, char *argv[])
{
	SafetyManager *instance = new SafetyManager();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SafetyManager::print_status()
{
	PX4_INFO("Safety Manager Status:");
	PX4_INFO("========================");

	// Print overall safety state
	PX4_INFO("Overall Safety Level: %d", static_cast<int>(_safety_state.overall_safety_level));
	PX4_INFO("Total Risk Score: %.2f", static_cast<double>(_safety_state.total_risk_score));

	// Print monitor states
	PX4_INFO("Speed State: %d", static_cast<int>(_safety_state.speed_state));
	PX4_INFO("Steering State: %d", static_cast<int>(_safety_state.steering_state));
	PX4_INFO("Stability State: %d", static_cast<int>(_safety_state.stability_state));
	PX4_INFO("Load State: %d", static_cast<int>(_safety_state.load_state));
	PX4_INFO("Communication State: %d", static_cast<int>(_safety_state.communication_state));

	// Print safety permits
	PX4_INFO("Safety Permits:");
	PX4_INFO("  Drive: %s", _safety_permits.drive_permitted ? "YES" : "NO");
	PX4_INFO("  High Speed: %s", _safety_permits.high_speed_permitted ? "YES" : "NO");
	PX4_INFO("  Boom Operation: %s", _safety_permits.boom_operation_permitted ? "YES" : "NO");
	PX4_INFO("  Bucket Operation: %s", _safety_permits.bucket_operation_permitted ? "YES" : "NO");
	PX4_INFO("  Steering: %s", _safety_permits.steering_permitted ? "YES" : "NO");

	// Print emergency response status
	PX4_INFO("Emergency Response Active: %s", _emergency_response.emergency_active ? "YES" : "NO");

	// Print performance metrics
	PX4_INFO("Performance:");
	PX4_INFO("  Cycle Time: %u us", _safety_performance.cycle_time_us);
	PX4_INFO("  Max Cycle Time: %u us", _safety_performance.max_cycle_time_us);
	PX4_INFO("  Fault Count: %u", _safety_performance.fault_count);
	PX4_INFO("  Recovery Count: %u", _safety_performance.recovery_count);

	return 0;
}

int SafetyManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SafetyManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Modular Safety Manager for wheel loader operations.

Provides comprehensive safety monitoring and fail-safe control for:
- Speed and acceleration monitoring
- Steering system monitoring
- Vehicle stability monitoring
- Load and center-of-gravity monitoring
- Communication link monitoring
- Hardware enable control
- Emergency response procedures

### Implementation
The safety manager uses a modular architecture with specialized monitor classes
for different subsystems, centralized permit management, and configurable
safety parameters. It runs at 50Hz and integrates with the PX4 uORB messaging
system for real-time safety control.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safety_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

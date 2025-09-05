#pragma once

#include <stdint.h>
#include <drivers/drv_hrt.h>

namespace safety_manager
{

// Safety system constants
static constexpr float SAFETY_CHECK_RATE_HZ = 50.0f;
static constexpr uint64_t SAFETY_CHECK_INTERVAL_US = 1000000 / static_cast<uint64_t>(SAFETY_CHECK_RATE_HZ);
static constexpr float DEFAULT_WHEEL_RADIUS_M = 0.5f;
static constexpr uint64_t COMMUNICATION_TIMEOUT_US = 5000000; // 5 seconds
static constexpr uint64_t SENSOR_TIMEOUT_US = 1000000; // 1 second
static constexpr uint64_t ZEROING_MODE_TIMEOUT_US = 30000000; // 30 seconds
static constexpr uint32_t MAX_LIMIT_SENSORS = 8;

// Safety levels
enum class SafetyLevel : uint8_t {
    NORMAL = 0,
    CAUTION = 1,
    WARNING = 2,
    CRITICAL = 3,
    EMERGENCY = 4
};

// Safety operation modes
enum class SafetyMode : uint8_t {
    MONITORING = 0,
    INTERVENTION = 1,
    OVERRIDE = 2,
    SHUTDOWN = 3
};

// System fault types (bit flags)
enum class FaultType : uint32_t {
    NONE = 0,
    SPEED_LIMIT = (1 << 0),
    STEERING_LIMIT = (1 << 1),
    COMMUNICATION = (1 << 2),
    HARDWARE = (1 << 3),
    SENSOR = (1 << 4),
    STABILITY = (1 << 5),
    LOAD = (1 << 6),
    TERRAIN = (1 << 7),
    ELECTRIC_ACTUATOR = (1 << 8),
    BOOM = (1 << 9),
    BUCKET = (1 << 10),
    ARTICULATION = (1 << 11),
    POWER = (1 << 12),
    THERMAL = (1 << 13),
    LIMIT_SENSOR = (1 << 14)
};

// Safety state information
struct SafetyState {
    SafetyLevel current_level{SafetyLevel::NORMAL};
    SafetyMode current_mode{SafetyMode::MONITORING};
    uint32_t active_faults{static_cast<uint32_t>(FaultType::NONE)};
    uint32_t fault_history{static_cast<uint32_t>(FaultType::NONE)};
    bool emergency_active{false};
    bool safety_override_active{false};
    uint64_t last_fault_time{0};
    uint32_t safety_violation_count{0};
    float overall_risk_factor{0.0f};
};

// Safety permits for various operations
struct SafetyPermits {
    bool motion_permitted{false};
    bool steering_permitted{false};
    bool autonomous_permitted{false};
    bool load_operation_permitted{false};
    bool high_speed_permitted{false};
    bool manual_override_active{false};
    bool electric_actuator_permitted{false};
    bool boom_operation_permitted{false};
    bool bucket_operation_permitted{false};
    bool articulation_permitted{false};
    bool engine_start_permitted{false};
    bool emergency_override_permitted{false};
    uint64_t permit_update_time{0};
};

// Emergency response state
struct EmergencyResponse {
    bool emergency_stop_commanded{false};
    bool controlled_stop_active{false};
    bool emergency_shutdown_active{false};
    bool backup_systems_active{false};
    uint64_t emergency_start_time{0};
    uint32_t emergency_activations{0};
    float emergency_deceleration_rate{5.0f}; // m/s²
};

// Performance monitoring metrics
struct SafetyPerformance {
    float safety_check_time_ms{0.0f};
    uint32_t total_safety_checks{0};
    uint32_t safety_interventions{0};
    float intervention_response_time_ms{0.0f};
    float safety_system_availability{1.0f};
    uint64_t last_performance_update{0};
};

// Hardware enable management configuration
struct HardwareEnableConfig {
    uint32_t gpio_enable{0};
    bool initialized{false};
    bool enabled{false};
    bool emergency_stop{false};
    bool safety_override{false};
    uint64_t last_limit_check{0};
    uint64_t last_enable_pub_time{0};

    struct LimitConfig {
        uint8_t min_limit_instance{255};
        uint8_t max_limit_instance{255};
        bool allow_into_min{false};
        bool allow_into_max{false};
    } limit_config[2]; // For H-bridge channels 0 and 1
};

// Utility functions for fault handling
inline uint32_t operator|(uint32_t lhs, FaultType rhs) {
    return lhs | static_cast<uint32_t>(rhs);
}

inline uint32_t operator&(uint32_t lhs, FaultType rhs) {
    return lhs & static_cast<uint32_t>(rhs);
}

inline uint32_t operator~(FaultType fault) {
    return ~static_cast<uint32_t>(fault);
}

inline bool has_fault(uint32_t faults, FaultType fault) {
    return (faults & static_cast<uint32_t>(fault)) != 0;
}

inline void set_fault(uint32_t &faults, FaultType fault) {
    faults |= static_cast<uint32_t>(fault);
}

inline void clear_fault(uint32_t &faults, FaultType fault) {
    faults &= ~static_cast<uint32_t>(fault);
}

} // namespace safety_manager

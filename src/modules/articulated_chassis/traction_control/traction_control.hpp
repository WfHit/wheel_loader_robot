/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/pid/PID.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>

// uORB subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_control_mode.h>
// #include <uORB/topics/chassis_control_command.h>  // TODO: Fix missing header
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_quad_encoder.h>

// uORB publications
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/wheel_setpoint.h>
#include <uORB/topics/steering_setpoint.h>
#include <uORB/topics/hbridge_command.h>

using namespace matrix;

class TractionControl final : public ModuleBase<TractionControl>, public ModuleParams,
    public px4::ScheduledWorkItem
{
public:
    TractionControl();
    ~TractionControl() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    /**
     * Slip estimation for articulated vehicle
     */
    struct SlipEstimate {
        float longitudinal{0.0f};      // Longitudinal slip ratio (-1 to 1)
        float lateral{0.0f};           // Lateral slip estimate (rad)
        float confidence{0.0f};        // Estimation confidence (0-1)
        uint64_t timestamp{0};
    };

    struct AxleState {
        float motor_speed{0.0f};       // Motor speed from encoder (rad/s)
        float wheel_speed{0.0f};       // Estimated wheel speed (rad/s)
        float velocity_cmd{0.0f};      // Velocity command to wheel controller (m/s)
        float force_cmd{0.0f};         // Force command to wheel controller (N)
        SlipEstimate slip;
        bool is_slipping{false};       // Slip detection flag
    };

    struct VehicleState {
        Vector3f velocity_body{};      // Body frame velocity from EKF (m/s)
        Vector3f acceleration_body{};  // Body frame acceleration (m/s²)
        float yaw_rate{0.0f};         // Yaw rate (rad/s)
        float articulation_angle{0.0f}; // Current articulation angle (rad)
        float ground_speed{0.0f};      // Estimated ground speed (m/s)
        float sideslip_angle{0.0f};   // Vehicle sideslip angle (rad)
    };

    // State estimation methods
    void update_vehicle_state();
    void update_motor_feedback();
    void estimate_axle_slip();
    void estimate_ground_conditions();

    // Slip calculation for articulated vehicle
    SlipEstimate calculate_axle_slip(
        float motor_speed,
        float vehicle_speed,
        float articulation_angle,
        bool is_front_axle
    );

    // Traction control algorithms
    void compute_velocity_commands();
    void compute_force_distribution();
    void compute_steering_compensation();

    // Force/torque distribution
    void limit_wheel_forces();

    // Articulated steering compensation
    float compute_articulation_correction();
    float calculate_kinematic_steering_rate(float desired_yaw_rate);

    // Safety features
    void detect_wheel_stall();
    void handle_excessive_slip();

    // Helper functions
    float estimate_friction_coefficient();
    void publish_wheel_commands();
    void publish_steering_command();

    // State variables
    AxleState _front_axle{};
    AxleState _rear_axle{};
    VehicleState _vehicle{};

    // Control setpoints from trajectory follower
    float _desired_velocity{0.0f};     // Desired forward velocity (m/s)
    float _desired_yaw_rate{0.0f};     // Desired yaw rate (rad/s)
    float _desired_force{0.0f};        // Desired total traction force (N)

    // Control outputs for wheel controller
    float _front_velocity_cmd{0.0f};   // Front wheel velocity command (m/s)
    float _rear_velocity_cmd{0.0f};    // Rear wheel velocity command (m/s)
    float _front_force_cmd{0.0f};      // Front axle force command (N)
    float _rear_force_cmd{0.0f};       // Rear axle force command (N)
    float _articulation_cmd{0.0f};     // Articulation angle command (rad)

    // Control parameters
    struct ControlParams {
        float target_slip_ratio{0.08f};     // Target slip for maximum traction
        float max_slip_ratio{0.15f};        // Maximum allowed slip before intervention
        float max_lateral_slip{0.1f};       // Maximum lateral slip (rad)
        float force_distribution{0.4f};     // Default front force ratio
        float stall_detection_threshold{0.05f}; // Wheel speed threshold for stall
    } _control_params;

    // Surface and material properties
    float _estimated_friction{0.7f};   // Estimated friction coefficient

    enum class SurfaceType {
        HARD_ROCK = 0,
        GRAVEL = 1,
        LOOSE_SOIL = 2,
        MUD = 3,
        UNKNOWN = 4
    } _surface_type{SurfaceType::GRAVEL};

    // PID controllers
    PID _slip_controller_front;
    PID _slip_controller_rear;
    PID _yaw_rate_controller;

    // Subscriptions
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    // uORB::Subscription _chassis_control_command_sub{ORB_ID(chassis_control_command)};  // TODO: Fix missing header
    uORB::SubscriptionMultiArray<sensor_quad_encoder_s> _encoder_sub{ORB_ID::sensor_quad_encoder};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

    // Publications
    uORB::PublicationMulti<wheel_setpoint_s> _wheel_setpoint_front_pub{ORB_ID(wheel_setpoint)};
    uORB::PublicationMulti<wheel_setpoint_s> _wheel_setpoint_rear_pub{ORB_ID(wheel_setpoint)};
    uORB::Publication<steering_setpoint_s> _steering_setpoint_pub{ORB_ID(steering_setpoint)};

    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::TC_SLIP_P>) _param_slip_p,
        (ParamFloat<px4::params::TC_SLIP_I>) _param_slip_i,
        (ParamFloat<px4::params::TC_SLIP_D>) _param_slip_d,
        (ParamFloat<px4::params::TC_TARGET_SLIP>) _param_target_slip,
        (ParamFloat<px4::params::TC_MAX_SLIP>) _param_max_slip,
        (ParamFloat<px4::params::TC_FORCE_DIST>) _param_force_distribution,
        (ParamFloat<px4::params::TC_WHEEL_RADIUS>) _param_wheel_radius,
        (ParamFloat<px4::params::TC_GEAR_RATIO>) _param_gear_ratio,
        (ParamFloat<px4::params::TC_VEHICLE_MASS>) _param_vehicle_mass,
        (ParamFloat<px4::params::TC_WHEELBASE>) _param_wheelbase,
        (ParamFloat<px4::params::TC_MAX_ARTIC>) _param_max_articulation,
        (ParamFloat<px4::params::TC_STALL_THRESH>) _param_stall_threshold,
        (ParamFloat<px4::params::TC_MAX_FORCE>) _param_max_force,
        (ParamInt<px4::params::TC_ENABLE>) _param_enable
    )

    // Performance monitoring
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _slip_detect_perf{perf_alloc(PC_COUNT, MODULE_NAME": slip events")};
    perf_counter_t _stall_detect_perf{perf_alloc(PC_COUNT, MODULE_NAME": stall events")};

    // Module state
    bool _traction_control_active{false};
    hrt_abstime _last_update{0};
    hrt_abstime _last_slip_event{0};
    hrt_abstime _last_stall_event{0};

    // Constants
    static constexpr float UPDATE_RATE_HZ = 50.0f;
    static constexpr float MIN_GROUND_SPEED = 0.1f;
    static constexpr float ARTICULATION_RATE_LIMIT = 0.5f;
};

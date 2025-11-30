#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include "ArraySequence.h"
#include "NewLazySequence.h"

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vector3() = default;
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vector3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
};

inline Vector3 ClampVector(const Vector3& value, double min_val, double max_val) {
    return {
        std::clamp(value.x, min_val, max_val),
        std::clamp(value.y, min_val, max_val),
        std::clamp(value.z, min_val, max_val)
    };
}

struct Orientation {
    double pitch{0.0};
    double roll{0.0};
    double yaw{0.0};
};

struct ShipPose {
    Vector3 position;
    Orientation orientation;
};

struct ShipKinematics {
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 angular_velocity;
    Vector3 angular_acceleration;
};

struct ShipState {
    ShipPose pose;
    ShipKinematics motion;
    double dt{0.02};
    size_t step{0};
};

struct Environment {
    Vector3 gravity{0.0, 0.0, -9.81};
    Vector3 wind_velocity{0.0, 0.0, 0.0};
};

struct OrientationLimits {
    double max_pitch{0.5};
    double max_roll{0.5};
    double max_yaw{3.14159};
};

struct ThrustProfile {
    Vector3 positive{15000.0, 15000.0, 40000.0};
    Vector3 negative{12000.0, 12000.0, 0.0};
};

struct AttitudeThrustProfile {
    Vector3 positive{5000.0, 5000.0, 3000.0};
    Vector3 negative{5000.0, 5000.0, 3000.0};
};

struct ShipParameters {
    ThrustProfile thrust;
    AttitudeThrustProfile attitude_thrust;
    Vector3 angular_rate_limit{0.1, 0.1, 0.2};
    OrientationLimits orientation_limits;
    Environment environment;
    double mass{2000.0};
    double default_dt{0.02};
};

struct LandingTarget {
    ShipPose pose;
    ShipKinematics motion;
};

inline LandingTarget DefaultLandingTarget() {
    LandingTarget target{};
    target.pose.position = {0.0, 0.0, 0.0};
    target.pose.orientation = {0.0, 0.0, 0.0};
    target.motion.velocity = {0.0, 0.0, 0.0};
    target.motion.acceleration = {0.0, 0.0, 0.0};
    target.motion.angular_velocity = {0.0, 0.0, 0.0};
    target.motion.angular_acceleration = {0.0, 0.0, 0.0};
    return target;
}

struct ControlInput {
    Vector3 thrust_ratio;
    Vector3 angular_thrust_ratio;
};

using ControlPolicy = std::function<ControlInput(const ShipState&,
                                                 const ShipParameters&,
                                                 const LandingTarget&)>;

inline Vector3 AxisAcceleration(const Vector3& ratio, const ShipParameters& params) {
    const auto compute = [&](double value, double pos_limit, double neg_limit) -> double {
        double limit = value >= 0.0 ? pos_limit : neg_limit;
        if (limit == 0.0 || params.mass <= 0.0) return 0.0;
        return (limit * value) / params.mass;
    };

    return {
        compute(ratio.x, params.thrust.positive.x, params.thrust.negative.x),
        compute(ratio.y, params.thrust.positive.y, params.thrust.negative.y),
        compute(ratio.z, params.thrust.positive.z, params.thrust.negative.z)
    };
}

inline Vector3 AxisAngularAcceleration(const Vector3& ratio, const ShipParameters& params) {
    const auto compute = [&](double value, double pos_limit, double neg_limit) -> double {
        double limit = value >= 0.0 ? pos_limit : neg_limit;
        if (limit == 0.0 || params.mass <= 0.0) return 0.0;
        return (limit * value) / params.mass;
    };

    return {
        compute(ratio.x, params.attitude_thrust.positive.x, params.attitude_thrust.negative.x),
        compute(ratio.y, params.attitude_thrust.positive.y, params.attitude_thrust.negative.y),
        compute(ratio.z, params.attitude_thrust.positive.z, params.attitude_thrust.negative.z)
    };
}

inline ControlInput DefaultControlLaw(const ShipState& state,
                                      const ShipParameters& params,
                                      const LandingTarget& target) {
    constexpr double k_pos = 0.2;
    constexpr double k_vel = 0.4;
    constexpr double k_angle = 0.5;
    constexpr double k_rate = 0.3;

    Vector3 pos_error = target.pose.position - state.pose.position;
    Vector3 vel_error = target.motion.velocity - state.motion.velocity;

    Vector3 desired_accel = pos_error * k_pos + vel_error * k_vel;
    desired_accel += target.motion.acceleration;
    desired_accel -= params.environment.gravity;

    const auto axis_ratio = [&](double desired, double pos_thr, double neg_thr) -> double {
        double limit_force = desired >= 0.0 ? pos_thr : neg_thr;
        if (params.mass <= 0.0 || limit_force == 0.0) return 0.0;
        double achievable = limit_force / params.mass;
        if (achievable == 0.0) return 0.0;
        return std::clamp(desired / achievable, -1.0, 1.0);
    };

    ControlInput input{};
    input.thrust_ratio = {
        axis_ratio(desired_accel.x, params.thrust.positive.x, params.thrust.negative.x),
        axis_ratio(desired_accel.y, params.thrust.positive.y, params.thrust.negative.y),
        axis_ratio(desired_accel.z, params.thrust.positive.z, params.thrust.negative.z)
    };

    Orientation target_orientation = target.pose.orientation;
    Orientation current = state.pose.orientation;

    Vector3 angle_error{
        -state.pose.orientation.pitch,  // target pitch = 0
        -state.pose.orientation.roll,   // target roll = 0
        0.0                             // yaw orientation not enforced
    };
    Vector3 rate_error{
        -state.motion.angular_velocity.x,
        -state.motion.angular_velocity.y,
        -state.motion.angular_velocity.z
    };

    Vector3 desired_ang_accel = angle_error * k_angle + rate_error * k_rate;

    const auto axis_ratio_ang = [&](double desired, double pos_thr, double neg_thr) -> double {
        double limit_force = desired >= 0.0 ? pos_thr : neg_thr;
        if (params.mass <= 0.0 || limit_force == 0.0) return 0.0;
        double achievable = limit_force / params.mass;
        if (achievable == 0.0) return 0.0;
        double ratio = desired / achievable;
        return std::clamp(ratio, -1.0, 1.0);
    };

    input.angular_thrust_ratio = {
        axis_ratio_ang(desired_ang_accel.x,
                       params.attitude_thrust.positive.x,
                       params.attitude_thrust.negative.x),
        axis_ratio_ang(desired_ang_accel.y,
                       params.attitude_thrust.positive.y,
                       params.attitude_thrust.negative.y),
        axis_ratio_ang(desired_ang_accel.z,
                       params.attitude_thrust.positive.z,
                       params.attitude_thrust.negative.z)
    };

    return input;
}

class LandingControlSystem {
public:
    LandingControlSystem(const ShipParameters& params,
                         const ShipState& initial_state,
                         LandingTarget target = DefaultLandingTarget(),
                         ControlPolicy policy = DefaultControlLaw)
        : params_(params),
          target_(std::move(target)),
          control_policy_(std::move(policy)),
          base_state_(initial_state),
          current_index_(0)
    {
        InitializeSequence();
    }

    void SetParameters(const ShipParameters& params) {
        params_ = params;
        InitializeSequence();
    }

    void SetLandingTarget(const LandingTarget& target) {
        target_ = target;
        InitializeSequence();
    }

    void SetControlPolicy(const ControlPolicy& policy) {
        control_policy_ = policy ? policy : DefaultControlLaw;
        InitializeSequence();
    }

    void SetInitialState(const ShipState& state) {
        base_state_ = state;
        InitializeSequence();
    }

    ShipState CurrentState() const {
        if (!trajectory_) {
            throw std::runtime_error("Trajectory is not initialized");
        }
        return trajectory_->Get(static_cast<int>(current_index_));
    }

    ShipState Step() {
        current_index_++;
        if (!trajectory_) {
            throw std::runtime_error("Trajectory is not initialized");
        }
        return trajectory_->Get(static_cast<int>(current_index_));
    }

    void Reset() {
        current_index_ = 0;
        InitializeSequence();
    }

private:
    std::unique_ptr<LazySequence<ShipState>> trajectory_;
    ShipParameters params_;
    LandingTarget target_;
    ControlPolicy control_policy_;
    ShipState base_state_;
    size_t current_index_;

    static Orientation ClampOrientation(const Orientation& orientation,
                                        const OrientationLimits& limits) {
        Orientation clamped;
        clamped.pitch = std::clamp(orientation.pitch,
                                   -limits.max_pitch,
                                   limits.max_pitch);
        clamped.roll = std::clamp(orientation.roll,
                                  -limits.max_roll,
                                  limits.max_roll);
        clamped.yaw = std::clamp(orientation.yaw,
                                 -limits.max_yaw,
                                 limits.max_yaw);
        return clamped;
    }

    ShipState Integrate(const ShipState& current, const ControlInput& input) const {
        double dt = current.dt > 0.0 ? current.dt : params_.default_dt;

        Vector3 clamped_ratio = ClampVector(input.thrust_ratio, -1.0, 1.0);
        Vector3 thrust_accel = AxisAcceleration(clamped_ratio, params_);
        Vector3 total_accel = thrust_accel + params_.environment.gravity;

        Vector3 next_velocity = current.motion.velocity + total_accel * dt;
        Vector3 wind_displacement = params_.environment.wind_velocity * dt;
        Vector3 next_position = current.pose.position + next_velocity * dt + wind_displacement;

        Vector3 clamped_angular_ratio = ClampVector(input.angular_thrust_ratio, -1.0, 1.0);
        Vector3 angular_accel = AxisAngularAcceleration(clamped_angular_ratio, params_);
        Vector3 next_ang_velocity = current.motion.angular_velocity + angular_accel * dt;
        next_ang_velocity.x = std::clamp(next_ang_velocity.x, -params_.angular_rate_limit.x, params_.angular_rate_limit.x);
        next_ang_velocity.y = std::clamp(next_ang_velocity.y, -params_.angular_rate_limit.y, params_.angular_rate_limit.y);
        next_ang_velocity.z = std::clamp(next_ang_velocity.z, -params_.angular_rate_limit.z, params_.angular_rate_limit.z);

        Orientation next_orientation{
            current.pose.orientation.pitch + next_ang_velocity.x * dt,
            current.pose.orientation.roll + next_ang_velocity.y * dt,
            current.pose.orientation.yaw + next_ang_velocity.z * dt
        };

        ShipState next = current;
        next.pose.position = next_position;
        next.pose.orientation = ClampOrientation(next_orientation, params_.orientation_limits);
        next.motion.velocity = next_velocity;
        next.motion.acceleration = total_accel;
        next.motion.angular_velocity = next_ang_velocity;
        next.motion.angular_acceleration = angular_accel;
        next.step = current.step + 1;
        next.dt = dt;
        return next;
    }

    void InitializeSequence() {
        ArraySequence<ShipState>* seed = new ArraySequence<ShipState>();
        seed->Append(base_state_);

        auto generator = [this](const std::deque<ShipState>& window) -> ShipState {
            if (window.empty()) {
                throw std::runtime_error("Generator window is empty");
            }
            const ShipState& current = window.back();
            ControlInput input = control_policy_(current, params_, target_);
            return Integrate(current, input);
        };

        trajectory_ = std::make_unique<LazySequence<ShipState>>(generator, seed, 1);
        delete seed;
        current_index_ = 0;
    }
};



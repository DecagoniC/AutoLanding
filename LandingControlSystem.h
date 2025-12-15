#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <cmath>

#include "ArraySequence.h"
#include "NewLazySequence.h"

enum class LandingStatus {
    InFlight = 0,
    Landed   = 1,
    Crashed  = 2
};

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
    target.pose.position = {-1000.0, 2000.0, 100.0};
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
    // Динамические коэффициенты по положению и скорости,
    // зависящие от текущего расстояния до цели и модуля скорости.
    // Для вертикали (ось Z) используем динамические коэффициенты.
    constexpr double k_pos_min   = 0.01;
    constexpr double k_pos_max   = 0.08;
    constexpr double k_vel_min   = 0.5;
    constexpr double k_vel_max   = 2.5;  // Верхняя граница демпфирования по скорости
    constexpr double k_angle     = 0.5;
    constexpr double k_rate      = 0.3;

    // Оценка расстояния до цели (по вертикали и горизонтали).
    const double dx = state.pose.position.x - target.pose.position.x;
    const double dy = state.pose.position.y - target.pose.position.y;
    const double dz = state.pose.position.z - target.pose.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double alt = std::abs(dz);  // Высота над целью

    // Оценка текущей скорости.
    const double vx = state.motion.velocity.x - target.motion.velocity.x;
    const double vy = state.motion.velocity.y - target.motion.velocity.y;
    const double vz = state.motion.velocity.z - target.motion.velocity.z;
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    const double vert_speed = std::abs(vz);  // Вертикальная скорость по модулю

    // Нормированные величины в диапазоне [0, 1] без жёсткого параметра «типичной высоты».
    // dist_norm растёт от 0 (в точке цели) до ~1 при больших расстояниях.
    const double dist_norm  = 1.0 - 1.0 / (1.0 + dist);
    // «Нормальная» характерная скорость увеличена до 300 м/с,
    // чтобы при тех же скоростях демпфирование было слабее и спуск происходил быстрее.
    const double speed_norm = std::clamp(speed / 300.0,  0.0, 1.0);

    // Специальная логика для финальной фазы посадки (низкая высота) по оси Z.
    // На малых высотах приоритет — гашение вертикальной скорости.
    constexpr double critical_altitude = 50.0;   // Критическая высота для усиления торможения
    constexpr double final_altitude    = 8.0;    // Финальная фаза посадки
    
    double alt_factor = 1.0;
    if (alt < critical_altitude) {
        // На малых высотах увеличиваем коэффициент демпфирования скорости,
        // но не столь резко, чтобы посадка не была слишком медленной.
        if (alt < final_altitude) {
            alt_factor = 2.0;
        } else {
            alt_factor = 1.0 + 1.5 * (1.0 - alt / critical_altitude);  // Плавное увеличение
        }
    }

    // Учитываем вертикальную скорость: чем она больше, тем сильнее нужно тормозить,
    // но с более мягким ростом, чтобы не переусердствовать.
    const double vert_speed_factor = 1.0 + std::clamp(vert_speed / 3.0, 0.0, 1.5);

    // Далеко от цели: слабый по положению, ближе — сильнее (по Z).
    // Используем квадратичную зависимость от нормированного расстояния,
    // чтобы влияние расстояния на тягу было более нелинейным и чувствительным.
    const double dist_weight    = dist_norm * dist_norm;         // квадратично по расстоянию
    double k_pos = k_pos_min + dist_weight * (k_pos_max - k_pos_min);
    if (alt < critical_altitude) {
        k_pos *= 0.3;  // Ослабляем позиционный компонент на малых высотах
    }
    
    // Коэффициент демпфирования скорости по Z: зависит от скорости, высоты и вертикальной скорости.
    const double k_vel_z = (k_vel_min + speed_norm * (k_vel_max - k_vel_min)) * alt_factor * vert_speed_factor;

    Vector3 pos_error = target.pose.position - state.pose.position;

    // Для компенсации ветра по X/Y хотим, чтобы наземная скорость была близка к нулю.
    // Поскольку интегратор добавляет снос wind_velocity в позицию, нам нужно,
    // чтобы скорость аппарата относительно среды была ≈ -wind_velocity в проекциях X/Y.
    Vector3 desired_vel = target.motion.velocity;
    desired_vel.x -= params.environment.wind_velocity.x;
    desired_vel.y -= params.environment.wind_velocity.y;
    Vector3 vel_error = desired_vel - state.motion.velocity;

    // Отдельные настройки по осям:
    // - по X/Y: более "жёсткий" PD, чтобы компенсировать ветер и не улетать от цели;
    // - по Z: текущая динамическая схема.
    constexpr double k_pos_xy_base = 0.03;
    constexpr double k_vel_xy_base = 0.9;

    // Квадратичная зависимость усиления по X/Y от расстояния до цели.
    // При больших расстояниях позиционный и скоростной компоненты сильнее,
    // при приближении к цели — ослабляются.
    const double k_pos_xy = k_pos_xy_base * (1.0 + dist_weight);
    const double k_vel_xy = k_vel_xy_base * (1.0 + dist_weight);

    Vector3 desired_accel{};
    desired_accel.x = pos_error.x * k_pos_xy + vel_error.x * k_vel_xy;
    desired_accel.y = pos_error.y * k_pos_xy + vel_error.y * k_vel_xy;
    desired_accel.z = pos_error.z * k_pos      + vel_error.z * k_vel_z;
    desired_accel += target.motion.acceleration;
    desired_accel -= params.environment.gravity;

    const auto axis_ratio = [&](double desired, double pos_thr, double neg_thr) -> double {
        double limit_force = desired >= 0.0 ? pos_thr : neg_thr;
        if (params.mass <= 0.0 || limit_force == 0.0) return 0.0;
        double achievable = limit_force / params.mass;
        if (achievable == 0.0) return 0.0;
        return std::clamp(desired / achievable, -1.0, 1.0);
    };

    // Области торможения по координатам.
    // Вне этих зон корабль разгоняется к цели с максимальной доступной тягой (bang-bang),
    // внутри зон — используется более мягкий PD‑режим (desired_accel).
    constexpr double brake_dist_x = 300.0;  // м по X
    constexpr double brake_dist_y = 300.0;  // м по Y
    // Увеличенная область торможения по высоте: начинаем гасить скорость выше.
    constexpr double brake_dist_z = 800.0;  // м по Z

    auto cruise_ratio_for_axis = [&](double delta) -> double {
        if (std::abs(delta) < 1e-6) return 0.0;
        return (delta > 0.0) ? 1.0 : -1.0; // максимальная тяга к цели
    };

    ControlInput input{};

    // X‑ось
    if (std::abs(pos_error.x) > brake_dist_x) {
        input.thrust_ratio.x = cruise_ratio_for_axis(pos_error.x);
    } else {
        input.thrust_ratio.x =
            axis_ratio(desired_accel.x, params.thrust.positive.x, params.thrust.negative.x);
    }

    // Y‑ось
    if (std::abs(pos_error.y) > brake_dist_y) {
        input.thrust_ratio.y = cruise_ratio_for_axis(pos_error.y);
    } else {
        input.thrust_ratio.y =
            axis_ratio(desired_accel.y, params.thrust.positive.y, params.thrust.negative.y);
    }

    // Z‑ось: режим «разгон – торможение» с прогнозом остановочного пути.
    {
        double dz = state.pose.position.z - target.pose.position.z; // >0 выше цели
        double vz = state.motion.velocity.z - target.motion.velocity.z; // <0 при падении вниз

        // Максимальное доступное ускорение вверх (при thrust_ratio.z = 1).
        double a_thrust_max_z = AxisAcceleration({0.0, 0.0, 1.0}, params).z;
        // Реальное тормозное a: тяга вверх минус гравитация (g < 0).
        double a_brake = a_thrust_max_z + params.environment.gravity.z;
        a_brake = std::max(1e-3, a_brake); // защитимся от деления на ноль

        double v_abs = std::abs(vz);
        double d_stop = (v_abs * v_abs) / (2.0 * a_brake); // v^2 / (2a)
        constexpr double k_safety = 1.3; // запас по расстоянию

        bool descending = (dz > 0.0 && vz < 0.0); // выше цели и падаем вниз
        bool need_brake = descending && (dz <= d_stop * k_safety);

        if (!descending) {
            // Не в режиме нормального снижения — используем мягкий PD‑режим.
            input.thrust_ratio.z =
                axis_ratio(desired_accel.z, params.thrust.positive.z, params.thrust.negative.z);
        } else if (!need_brake && std::abs(dz) > brake_dist_z) {
            // Далеко от цели и ещё рано тормозить — летим в насыщении к цели (вниз).
            input.thrust_ratio.z = cruise_ratio_for_axis(pos_error.z); // pos_error.z < 0 -> вниз
        } else {
            // Пора тормозить: тянем вверх почти с максимальной тягой.
            // Если уже сильно замедлились, даём PD для более мягкой подводки.
            if (v_abs > 5.0) {
                input.thrust_ratio.z = 1.0; // сильное торможение вверх
            } else {
                input.thrust_ratio.z =
                    axis_ratio(desired_accel.z, params.thrust.positive.z, params.thrust.negative.z);
            }
        }
    }

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

    // Доступ к целевой точке посадки (для логирования и анализа).
    LandingTarget GetLandingTarget() const {
        return target_;
    }

    // Интерфейс для получения статуса посадки по текущему состоянию корабля.
    // Не влияет на управление и не изменяет внутреннее состояние системы.
    LandingStatus GetStatus() const {
        return EvaluateStatus(CurrentState());
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

    LandingStatus EvaluateStatus(const ShipState& state) const {
        // Ошибки относительно целевой точки
        const double z        = state.pose.position.z;
        const double z_target = target_.pose.position.z;
        const double dx       = state.pose.position.x - target_.pose.position.x;
        const double dy       = state.pose.position.y - target_.pose.position.y;
        const double xy_dist  = std::sqrt(dx * dx + dy * dy);

        Vector3 vel_err{
            state.motion.velocity.x - target_.motion.velocity.x,
            state.motion.velocity.y - target_.motion.velocity.y,
            state.motion.velocity.z - target_.motion.velocity.z
        };
        Vector3 ang_vel_err{
            state.motion.angular_velocity.x - target_.motion.angular_velocity.x,
            state.motion.angular_velocity.y - target_.motion.angular_velocity.y,
            state.motion.angular_velocity.z - target_.motion.angular_velocity.z
        };
        Orientation ang_err{
            state.pose.orientation.pitch - target_.pose.orientation.pitch,
            state.pose.orientation.roll  - target_.pose.orientation.roll
            // yaw целенаправленно не ограничиваем жёстко
        };

        const double horiz_speed = std::sqrt(
            vel_err.x * vel_err.x +
            vel_err.y * vel_err.y
        );

        constexpr double kAltTol        = 0.5;  // допуск по высоте, м
        constexpr double kXYTol         = 5.0;  // допуск по горизонтали, м
        constexpr double kMaxLandV      = 0.5;  // допуск по линейной скорости, м/с
        constexpr double kMaxLandAngle  = 0.1;  // допуск по pitch/roll, рад
        constexpr double kMaxLandOmega  = 0.1;  // допуск по угловой скорости, рад/с

        const bool near_vertical =
            std::fabs(z - z_target) <= kAltTol;
        const bool near_lateral =
            xy_dist <= kXYTol;

        const bool small_linear_vel =
            std::fabs(vel_err.z) <= kMaxLandV &&
            horiz_speed          <= kMaxLandV;

        const bool small_angles =
            std::fabs(ang_err.pitch) <= kMaxLandAngle &&
            std::fabs(ang_err.roll)  <= kMaxLandAngle;

        const bool small_omegas =
            std::fabs(ang_vel_err.x) <= kMaxLandOmega &&
            std::fabs(ang_vel_err.y) <= kMaxLandOmega &&
            std::fabs(ang_vel_err.z) <= kMaxLandOmega;

        // Успешная посадка: рядом с целью по координатам и с малыми скоростями/углами.
        if (near_vertical && near_lateral &&
            small_linear_vel && small_angles && small_omegas) {
            return LandingStatus::Landed;
        }

        // Если мы близко к уровню цели, но скорости/углы слишком большие — считаем, что разбились.
        if (near_vertical && (!small_linear_vel || !small_angles)) {
            return LandingStatus::Crashed;
        }

        // Если сильно «пролетели» цель по высоте вниз — точно разбились.
        if (z < z_target - 2.0) {
            return LandingStatus::Crashed;
        }

        return LandingStatus::InFlight;
    }
};
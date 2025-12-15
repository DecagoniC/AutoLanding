#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "LandingControlSystem.h"

int main() {
    std::ofstream log("landing_log.txt");
    if (!log.is_open()) {
        std::cerr << "Failed to open landing_log.txt for writing.\n";
        return 1;
    }

    ShipParameters params;
    params.mass = 2200.0;
    params.thrust.positive = {20000.0, 20000.0, 50000.0};
    params.thrust.negative = {15000.0, 15000.0, 0.0};
    params.environment.gravity = {0.0, 0.0, -9.81};
    params.environment.wind_velocity = {5.0, 0.0, 0.0};
    params.orientation_limits = {0.35, 0.35, 3.14159};
    params.angular_rate_limit = {0.05, 0.05, 0.08};

    ShipState initial{};
    initial.pose.position = {0.0, 0.0, 6000.0};
    initial.motion.velocity = {1.8, 50, -60.0};
    initial.motion.acceleration = {0.3, 6, -10};
    initial.motion.angular_velocity = {0.02, -0.03, 0.01};
    initial.motion.angular_acceleration = {0.05, 0.07, 0.1};
    initial.pose.orientation = {0.30, -0.18, 0.2}; // небольшие углы pitch/roll/yaw на старте
    initial.dt = 0.05;

    LandingTarget target = DefaultLandingTarget();
 
    LandingControlSystem controller(params, initial, target);
 
    log << std::fixed << std::setprecision(2);
    log << "Simulating powered landing...\n";
    log << "Target pos=(" << target.pose.position.x << "," << target.pose.position.y << "," << target.pose.position.z << ")\n";
    log << "Step\tPos(x,y,z, m)\tAltitude(z, m)\tVerticalVel(dz/dt, m/s)\tHorizontalVel(sqrt(vx^2+vy^2), m/s)\tDistToTarget(m)\tPitch(rad)\tRoll(rad)\n";

    ShipState state = controller.CurrentState();
    auto horizontal_speed = [](const ShipState& s) {
        return std::sqrt(s.motion.velocity.x * s.motion.velocity.x +
                         s.motion.velocity.y * s.motion.velocity.y);
    };
    auto distance_to_target = [&controller](const ShipState& s) {
        LandingTarget target = controller.GetLandingTarget();
        double dx = s.pose.position.x - target.pose.position.x;
        double dy = s.pose.position.y - target.pose.position.y;
        double dz = s.pose.position.z - target.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    const auto log_state = [&log](const ShipState& s, double horiz_speed, double dist_to_target) {
        log << "step=" << s.step
            << " pos=(" << s.pose.position.x << "," << s.pose.position.y << "," << s.pose.position.z << ")"
            << " alt=" << s.pose.position.z << "m"
            << " v_vert=" << s.motion.velocity.z << "m/s"
            << " v_h=" << horiz_speed << "m/s"
            << " dist=" << dist_to_target << "m"
            << " pitch=" << s.pose.orientation.pitch
            << " roll=" << s.pose.orientation.roll
            << " yaw=" << s.pose.orientation.yaw
            << " omega=(" << s.motion.angular_velocity.x << "," << s.motion.angular_velocity.y << "," << s.motion.angular_velocity.z << ")"
            << '\n';
    };

    log_state(state, horizontal_speed(state), distance_to_target(state));

    const size_t max_steps = 4000;
    for (size_t i = 0; i < max_steps; ++i) {
        state = controller.Step();
        log_state(state, horizontal_speed(state), distance_to_target(state));

        LandingStatus status = controller.GetStatus();

        // Прекращаем симуляцию, когда полёт считается завершённым.
        if (status == LandingStatus::Landed) {
            log << "Flight result: successful landing.\n";
            break;
        } else if (status == LandingStatus::Crashed) {
            log << "Flight result: crash landing.\n";
            break;
        }
    }

    if (controller.GetStatus() == LandingStatus::InFlight) {
        log << "Flight result: incomplete (max steps reached without landing).\n";
    }

    std::cout << "Telemetry saved to landing_log.txt\n";
    return 0;
}
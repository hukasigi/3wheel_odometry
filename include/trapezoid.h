#pragma once

#include <algorithm>
#include <cmath>

double updateVelocityProfile(double target_pos, double now_pos, double current_speed, double max_speed, double acceleration,
                             double dt) {
    const double error = target_pos - now_pos;

    constexpr double POSITION_THRESHOLD = 1.0;

    if (fabs(error) < POSITION_THRESHOLD) {
        return 0.0;
    }

    const double direction = sign(error);

    // 現在速度が目標方向と逆なら、まず停止する
    if (current_speed * direction < 0.0) {
        double next_speed = current_speed + direction * acceleration * dt;

        // 0を跨がないようにする
        if (next_speed * current_speed < 0.0) {
            next_speed = 0.0;
        }

        return next_speed;
    }

    // 目標方向に進んでいる場合
    const double stop_distance = (current_speed * current_speed) / (2.0 * acceleration);

    double next_speed = current_speed;

    if (fabs(error) > stop_distance) {
        // 加速
        next_speed += direction * acceleration * dt;
    } else {
        // 減速
        next_speed -= direction * acceleration * dt;
    }

    next_speed = constrain(next_speed, -max_speed, max_speed);

    return next_speed;
}

double updateAngleVelocityProfile(double target_deg, double now_deg, double current_speed, double max_speed,
                                  double max_acceleration, double dt) {
    const double error = wrapAngle(target_deg - now_deg);

    constexpr double ANGLE_THRESHOLD = 0.5;

    if (fabs(error) < ANGLE_THRESHOLD) {
        return 0.0;
    }

    const double stop_angle = (current_speed * current_speed) / (2.0 * max_acceleration);

    double next_speed = current_speed;

    if (fabs(error) > stop_angle) {

        next_speed += sign(error) * max_acceleration * dt;

    } else {

        next_speed -= sign(current_speed) * max_acceleration * dt;
    }

    next_speed = constrain(next_speed, -max_speed, max_speed);

    return next_speed;
}
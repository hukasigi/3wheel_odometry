#pragma once

#include <algorithm>
#include <cmath>

class TrapezoidProfile {
    public:
        TrapezoidProfile(double max_velocity, double max_acceleration)
            : max_velocity_(max_velocity), max_acceleration_(max_acceleration), position_(0.0), velocity_(0.0) {}

        void reset(double position = 0.0) {
            position_ = position;
            velocity_ = 0.0;
        }

        double update(double target, double dt) {
            if (dt <= 0.0) {
                return position_;
            }

            const double error = target - position_;

            // ほぼ目標位置なら停止
            if (std::abs(error) < 1e-6 && std::abs(velocity_) < 1e-6) {
                position_ = target;
                velocity_ = 0.0;
                return position_;
            }

            const double direction = (error > 0.0) ? 1.0 : -1.0;

            // 現在の速度から停止するまでに必要な距離
            const double stopping_distance = (velocity_ * velocity_) / (2.0 * max_acceleration_);

            // 減速を開始するべきか
            if (std::abs(error) <= stopping_distance) {
                velocity_ -= direction * max_acceleration_ * dt;
            } else {
                velocity_ += direction * max_acceleration_ * dt;
            }

            // 最大速度制限
            velocity_ = std::clamp(velocity_, -max_velocity_, max_velocity_);

            // 目標を通り越さないようにする
            const double next_position = position_ + velocity_ * dt;

            if ((direction > 0.0 && next_position >= target) || (direction < 0.0 && next_position <= target)) {

                position_ = target;
                velocity_ = 0.0;
            } else {
                position_ = next_position;
            }

            return position_;
        }

        double position() const { return position_; }

        double velocity() const { return velocity_; }

    private:
        double max_velocity_;
        double max_acceleration_;

        double position_;
        double velocity_;
};

class AngleTrapezoidProfile {
    public:
        AngleTrapezoidProfile(double max_velocity, double max_acceleration, double range = 360.0)
            : max_velocity_(max_velocity), max_acceleration_(max_acceleration), range_(range), position_(0.0), velocity_(0.0),
              initialized_(false) {}

        void reset(double position = 0.0) {
            position_    = position;
            velocity_    = 0.0;
            initialized_ = true;
        }

        double update(double target, double current, double dt) {
            if (dt <= 0.0) {
                return position_;
            }

            // 初回だけ現在角度をプロファイルの位置にする
            if (!initialized_) {
                position_    = current;
                velocity_    = 0.0;
                initialized_ = true;
            }

            // -------------------------
            // 最短方向の角度誤差
            // -------------------------
            double error = target - position_;

            error = std::fmod(error + range_ / 2.0, range_);

            if (error < 0.0) {
                error += range_;
            }

            error -= range_ / 2.0;

            // -------------------------
            // ほぼ到着
            // -------------------------
            if (std::abs(error) < 0.01 && std::abs(velocity_) < 0.01) {

                position_ = target;
                velocity_ = 0.0;

                return position_;
            }

            // -------------------------
            // 進行方向
            // -------------------------
            const double direction = (error >= 0.0) ? 1.0 : -1.0;

            // -------------------------
            // 停止するために必要な距離
            // v² = 2ad
            // -------------------------
            const double stopping_distance = (velocity_ * velocity_) / (2.0 * max_acceleration_);

            // -------------------------
            // 加速 / 減速
            // -------------------------
            if (std::abs(error) <= stopping_distance) {

                // 減速
                velocity_ -= direction * max_acceleration_ * dt;

            } else {

                // 加速
                velocity_ += direction * max_acceleration_ * dt;
            }

            // 最大速度
            velocity_ = std::clamp(velocity_, -max_velocity_, max_velocity_);

            // -------------------------
            // 位置更新
            // -------------------------
            position_ += velocity_ * dt;

            // 位置を 0～360 に収める
            position_ = std::fmod(position_, range_);

            if (position_ < 0.0) {
                position_ += range_;
            }

            return position_;
        }

        double position() const { return position_; }

        double velocity() const { return velocity_; }

    private:
        double max_velocity_;
        double max_acceleration_;
        double range_;

        double position_;
        double velocity_;

        bool initialized_;
};
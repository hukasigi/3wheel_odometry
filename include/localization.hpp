#pragma once

#include <cmath>

#include "constants.hpp"
#include "nnct/interfaces/incremental_encoder.hpp"

struct Position {
        double x;
        double y;
        double deg;

        Position() : x(0.0), y(0.0), deg(0.0) {}
        Position(double x_, double y_, double yaw_) : x(x_), y(y_), deg(yaw_) {}
};

class Odometry {
    public:
        Odometry(nnct::interfaces::IncrementalEncoder& encoder1, nnct::interfaces::IncrementalEncoder& encoder2,
                 nnct::interfaces::IncrementalEncoder& encoder3);

        void     begin();
        Position get_position() const;
        Position get_velocity() const;
        void     update(double dt);

    private:
        static bool Invert3x3(const double A[3][3], double invA[3][3]);
        void        _buildInverse();

        nnct::interfaces::IncrementalEncoder& encoder_1_;
        nnct::interfaces::IncrementalEncoder& encoder_2_;
        nnct::interfaces::IncrementalEncoder& encoder_3_;

        int32_t prev_count1_{0};
        int32_t prev_count2_{0};
        int32_t prev_count3_{0};

        double last_dc1_{0.0};
        double last_dc2_{0.0};
        double last_dc3_{0.0};

        Position position_{};
        Position velocity_{};

        bool   inv_ok_{false};
        double invA_[3][3]{};
};
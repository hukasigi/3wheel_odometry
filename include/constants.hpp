#pragma once
#include <Arduino.h>
// units: mm, deg, rad, mm/s

struct PidParam {
        double p_gain;
        double i_gain;
        double d_gain;
};

using pin_t = uint8_t;
using ch_t  = uint8_t;

const pin_t ENCODER_A_1 = 27;
const pin_t ENCODER_B_1 = 14;

const pin_t ENCODER_A_2 = 25;
const pin_t ENCODER_B_2 = 26;

const pin_t ENCODER_A_3 = 32;
const pin_t ENCODER_B_3 = 33;

const pin_t can_tx = 5;
const pin_t can_rx = 4;

constexpr double OD_RADIUS    = 30.0;
constexpr double DRIVE_RADIUS = 50;

constexpr double ROBOT_TO_ODO_RADIUS = 210.0;

constexpr int8_t   ENCODER_SIGN_1     = 1;
constexpr int8_t   ENCODER_SIGN_2     = 1;
constexpr int8_t   ENCODER_SIGN_3     = 1;
constexpr uint32_t ENCODER_RESOLUTION = 8192;

constexpr double        GEAR_RATIO       = 1.0;
static constexpr double DRIVE_GEAR_RATIO = 19.0 / 1.0; // モーター:ホイールの速度比

constexpr double COUNTS_PER_MM = (ENCODER_RESOLUTION * GEAR_RATIO) / (M_PI * OD_RADIUS * 2.0);

static constexpr double STEER_GEAR_RATIO_MOTOR_TO_STEER = 65.0 / 27.0;

static const int32_t CALIBRATING_DUTY = 150;

// 位置制御PIDゲイン
const struct PidParam PID_PARAM_X = {.p_gain = 1.0, .i_gain = 0.0, .d_gain = 0.0};
const struct PidParam PID_PARAM_Y = {.p_gain = 1.0, .i_gain = 0.0, .d_gain = 0.0};

const struct PidParam PID_PARAM_YAW = {.p_gain = 1., .i_gain = 0.0, .d_gain = 0.0};

// 許容誤差
constexpr double POSITION_TOLERANCE_MM = 10.0;
constexpr double YAW_TOLERANCE_RAD     = 5.0 * M_PI / 180.0;

// PID積分制限
constexpr double POSITION_INTEGRAL_MAX = 300.0;
constexpr double POSITION_INTEGRAL_MIN = -300.0;
constexpr double YAW_INTEGRAL_MAX      = 300.0;
constexpr double YAW_INTEGRAL_MIN      = -300.0;

const double SPEED_EPS = 1e-3; // 1 mm/s 程度のノイズは角度更新を行わない
// 2) 速度正規化（任意の物理上限に合わせる）
// 3輪の合計したときの速度、2400mm/sが最大
const double SHIFT_MAX_SPEED = 2400.0;

constexpr double MAX_ANGULAR_SPEED_DEG_S = 30.0;

constexpr double YAW_RANGE_DEG = 360.;

constexpr double MAX_SHIFT_ACCELERATION  = 1000.;
constexpr double MAX_ROTATE_ACCELERATION = 300.;

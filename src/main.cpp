#include "AnglePid.h"
#include "Pid.h"
#include "esp_can.hpp"
#include "localization.hpp"
#include "nnct/interfaces/incremental_encoder.hpp"
#include <Arduino.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <peer_link.h>
#include <vector>

using namespace nnct::interfaces;

IncrementalEncoder enc_1(ENCODER_A_1, ENCODER_B_1);
IncrementalEncoder enc_2(ENCODER_A_2, ENCODER_B_2);
IncrementalEncoder enc_3(ENCODER_A_3, ENCODER_B_3);

Odometry odometry(enc_1, enc_2, enc_3);

constexpr uint8_t WIFI_CHANNEL          = 14;
const peer_id_t   FROM_PEER_ID          = 0x11;
const peer_id_t   TO_PEER_ID            = 0x12;
constexpr uint8_t POSITION_MESSAGE_TYPE = 0x01;

int16_t target_x   = 0;
int16_t target_y   = 0;
int16_t target_deg = 0;

unsigned long  last          = micros();
const uint16_t CONTROL_CYCLE = 5000;

Position now_pos;
Position target_pos;

CanDriver can;

PositionPID x_pos_pid(PID_PARAM_X.p_gain, PID_PARAM_X.i_gain, PID_PARAM_X.d_gain, -wheel3_max_speed, wheel3_max_speed,
                      POSITION_INTEGRAL_MIN, POSITION_INTEGRAL_MAX);
PositionPID y_pos_pid(PID_PARAM_Y.p_gain, PID_PARAM_Y.i_gain, PID_PARAM_Y.d_gain, -wheel3_max_speed, wheel3_max_speed,
                      POSITION_INTEGRAL_MIN, POSITION_INTEGRAL_MAX);

AnglePID yayPID(PID_PARAM_YAY.p_gain, PID_PARAM_YAY.i_gain, PID_PARAM_YAY.d_gain, -MAX_ANGULAR_SPEED_DEG_S,
                MAX_ANGULAR_SPEED_DEG_S, YAW_INTEGRAL_MIN, YAW_INTEGRAL_MAX, YAW_RANGE_DEG);

std::vector<uint8_t> positionToPayload(const void* data) {
    const Position* position = static_cast<const Position*>(data);

    std::vector<uint8_t> payload(sizeof(Position));
    std::memcpy(payload.data(), position, sizeof(Position));

    return payload;
}

int16_t readInt16(const std::vector<uint8_t>& data, size_t index) {
    uint16_t value = static_cast<uint16_t>(data[index]) | (static_cast<uint16_t>(data[index + 1]) << 8);

    return static_cast<int16_t>(value);
}
void peer_link_recv_cb(const peer_id_t peer_id, const std::vector<struct Message>& messages) {

    for (const Message& message : messages) {
        if (message.type != POSITION_MESSAGE_TYPE) {
            continue;
        }

        if (message.data.size() < 6) {
            Serial.println("invalid target data");
            continue;
        }

        target_x   = readInt16(message.data, 0);
        target_y   = readInt16(message.data, 2);
        target_deg = readInt16(message.data, 4);

        target_pos.x   = target_x;
        target_pos.y   = target_y;
        target_pos.deg = target_deg;

        // ここに目標値を使った制御処理を記述
    }
}

void setup() {
    odometry.begin();
    if (!can.begin(1000000, can_tx, can_rx)) {
        Serial.println("can failed");
    }

    peer_link_task_init(WIFI_CHANNEL, FROM_PEER_ID);
}

void loop() {
    unsigned long now = micros();

    if (now - last < CONTROL_CYCLE) return;
    double dt = (now - last) * 1.e-6;
    last      = now;
    odometry.update(dt);

    now_pos = odometry.get_position();

    if (peer_link_is_peer_exist(TO_PEER_ID)) {
        Message message;
        message.type = POSITION_MESSAGE_TYPE;
        message.data = positionToPayload(&now_pos);

        std::vector<Message> messages{message};

        const esp_err_t result = peer_link_send(TO_PEER_ID, messages);

        if (result != ESP_OK) {
            Serial.printf("send error: %d\n", result);
        }
    }

    int16_t x_vec   = x_pos_pid.update(target_pos.x, now_pos.x, dt);
    int16_t y_vec   = y_pos_pid.update(target_pos.y, now_pos.y, dt);
    int16_t deg_vec = yayPID.update(target_pos.deg, now_pos.deg, dt);

    uint32_t id      = 0x300;
    uint8_t  data[6] = {
        static_cast<uint8_t>((x_vec >> 8) & 0xFF),   static_cast<uint8_t>(x_vec & 0xFF),

        static_cast<uint8_t>((y_vec >> 8) & 0xFF),   static_cast<uint8_t>(y_vec & 0xFF),

        static_cast<uint8_t>((deg_vec >> 8) & 0xFF), static_cast<uint8_t>(deg_vec & 0xFF),
    };

    can.sendStandard(id, data, sizeof(data));
    delay(10);
}
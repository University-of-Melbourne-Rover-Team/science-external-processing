#pragma once 

#include <WiFi.h>
#include <esp_now.h>
#include <cstring>

#include <WriteBufferFixedSize.h>
#include <ReadBufferFixedSize.h>
#include "generated/processing.h"

#include "gpio.h"
#include "ads_7828.h"
#include "drv.h"

esp_now_peer_info_t peer_info;
void on_data_recv(const esp_now_recv_info *info, const uint8_t *data, int len);
Recv::GpioResponse encode_gpio_response(bool enabled, uint16_t current);
Recv::MotorResponse encode_motor_response(bool fault, uint16_t in);

void setup_esp_now_comms() {
  WiFi.mode(WIFI_STA);

  if (!esp_now_init()) {
    return;
  }

  esp_now_register_recv_cb(on_data_recv);
}

void handle_motor_request(const Request::Motor& req, uint8_t idx);

void handle_incoming_request(const Request& req) {
  ctrl_heating(req.get_heating_element());

  const auto& motor_1 = req.get_motor_1();
  const auto& motor_2 = req.get_motor_2();
  handle_motor_request(motor_1, 0);
  handle_motor_request(motor_2, 1);
}

void handle_motor_request(const Request::Motor& req, uint8_t idx) {
  if (req.has_off()) {
    drv_stop_motor(idx);
  } else {
    const auto vel = req.get_velocity();
    float duty;

    if (vel < 0) {
      duty = 25.;
    } else if (vel > 0) {
      duty = 25.;
    } else {
      duty = 0.;
    }
    drv_set_motor_velocity(idx, duty);
  }
}

#define READ_BUF_SIZE 64
#define WRITE_BUF_SIZE 64
EmbeddedProto::ReadBufferFixedSize<READ_BUF_SIZE> read_buf;
EmbeddedProto::WriteBufferFixedSize<WRITE_BUF_SIZE> write_buf;
Request request;

const uint8_t empty_addr[6] = {0};

void on_data_recv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  read_buf.clear();


  if (!std::memcmp(peer_info.peer_addr, empty_addr, 6) && !esp_now_is_peer_exist(peer_info.peer_addr)) {
    auto buf = read_buf.get_data();
    memcpy(buf, data, len);

    read_buf.set_bytes_written(len);
    const auto err = request.deserialize(read_buf);

    if (err != EmbeddedProto::Error::NO_ERRORS) {
      return;
    }

    // set peer id to the first one with valid data.
    peer_info.channel = 0;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, info->src_addr, 6);

    if (esp_now_add_peer(&peer_info)) {
      memset(peer_info.peer_addr, 0, 6);
      return;
    }
  } else if (!std::memcmp(info->src_addr, peer_info.peer_addr, 6)) {
    auto buf = read_buf.get_data();
    memcpy(buf, data, len);

    read_buf.set_bytes_written(len);
    const auto err = request.deserialize(read_buf);

    if (err != EmbeddedProto::Error::NO_ERRORS) {
      return;
    }
  }
}

const Request& current_request() {
  return request;
}

Recv::MotorResponse encode_motor_response(bool fault, uint16_t in) {
  Recv::MotorResponse res;
  if (fault) {
    Recv::MotorResponse::Fault f;
    res.set_fault(f);
  } else {
    res.set_current_sense(in);
  }
  return res;
}

Recv::GpioResponse encode_gpio_response(bool enabled, uint16_t current) {
  Recv::GpioResponse res;
  if (!enabled) {
    Recv::GpioResponse::Off o;
    res.set_off(o);
  } else {
    res.set_sense_current(current);
  }
  return res;
}

Recv recv;
void try_send_data(const GpioReading& gpio, const AdsReading& ads, const bool pwr_on) {
  if (!std::memcmp(empty_addr, peer_info.peer_addr, 6)) {
    // no assigned peer addr yet
    return;
  }

  const auto mr_1 = encode_motor_response(gpio.bits.motor_driver_fault_1, ads.motor_driver_in_1);
  const auto mr_2 = encode_motor_response(gpio.bits.motor_driver_fault_2, ads.motor_driver_in_2);

  // gets the first 4 flags for the limit switches
  const auto flags = gpio.raw & 0b1111;

  const auto heating_on = request.get_heating_element();

  const auto heating = encode_gpio_response(heating_on, ads.heating_element_sense);
  const auto power = encode_gpio_response(pwr_on, ads.power_ctrl_sense);

  recv.set_limit_switches(flags);
  recv.set_heating_element(heating);
  recv.set_power(power);

  recv.set_motor_response_1(mr_1);
  recv.set_motor_response_2(mr_2);

  recv.set_temp_sense(0);

  write_buf.clear();
  const auto err = recv.serialize(write_buf);
  if (err != EmbeddedProto::Error::NO_ERRORS) {
    return;
  }

  const auto len = write_buf.get_size();
  const auto buf = write_buf.get_data();
  esp_now_send(peer_info.peer_addr, buf, len);
}


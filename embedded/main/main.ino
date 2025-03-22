#pragma once
#define analog_read(pin) analogRead(pin)
#define sleep_us(us) delayMicroseconds(us)
#define pin_mode(pin, kind) pinMode(pin, kind)
#define digital_write(pin, state) digitalWrite(pin, state)
#define digital_read(pin) digitalRead(pin)

#include "generated/assignment.h"
#include "generated/processing.h"
#include <WriteBufferFixedSize.h>
#include <ReadBufferFixedSize.h>
#include <Errors.h>

#include <gpio.h>
#include <ads_7828.h>
#include <drv.h>
#include <esp_comms.h>

//NOTE: EmbeddedProtobuf needs to be added as a library for this to work
//which the submodule needs to be cloned

//on mac its
//mkdir ~/Documents/Arduino/libraries/EmbeddedProto_Library
//cp EmbeddedProto/src/* ~/Documents/Arduino/libraries/EmbeddedProto_Library

//on linux its
//mkdir ~/Arduino/libraries/EmbeddedProto_Library
//cp EmbeddedProto/src/* ~/Arduino/libraries/EmbeddedProto_Library/

//idk what it is on windows. sorry!

bool power_enabled = false;
void try_enable_pwr_ctrl() {
  const bool ctrl_pwr = analog_read(OFFSIGIN) != 0.;

  if (ctrl_pwr && !power_enabled) {
    // do init
    power_enabled = true;
    digital_write(PWR_CTRL, HIGH);
  } else if (!ctrl_pwr && power_enabled) {
    // do uninit
    power_enabled = false;
    digital_write(PWR_CTRL, LOW);
  }
}

void setup() {
  setup_gpio();
  setup_spi();
  setup_i2c();
  setup_esp_now_comms();
  try_enable_pwr_ctrl();
}

AdsReading ads_reading;
GpioReading gpio_reading;

void loop() {
  try_enable_pwr_ctrl();

  if (!power_enabled) {
    return;
  }

  read_ads(&ads_reading);
  read_gpio(&gpio_reading);

  try_send_data(gpio_reading, ads_reading, power_enabled);
  delay(10);
}

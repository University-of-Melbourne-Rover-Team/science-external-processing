#pragma once

#include <assert.h>


#define PWR_CTRL D13
#define OFFSIGIN A0
void setup_power_control() {
  pin_mode(PWR_CTRL, OUTPUT);
  pin_mode(OFFSIGIN, INPUT);
}

#define NFAULT_MD1 D5
#define NFAULT_MD2 D6
void setup_motordriver_faults() {
  pin_mode(NFAULT_MD1, INPUT);
  pin_mode(NFAULT_MD2, INPUT);
}

#define LIMIT_4 D7
#define LIMIT_3 D9
#define LIMIT_2 D10
#define LIMIT_1 D11
void setup_limit_swtiches() {
  pin_mode(LIMIT_1, INPUT);
  pin_mode(LIMIT_2, INPUT);
  pin_mode(LIMIT_3, INPUT);
  pin_mode(LIMIT_4, INPUT);
}

#define HE_CTRL D12
void setup_heating_element() {
  pin_mode(HE_CTRL, OUTPUT);
}

void setup_gpio() {
  setup_power_control();
  setup_limit_swtiches();
  setup_heating_element();
  setup_motordriver_faults();
}

typedef struct gpio_reading {
  union {
    uint8_t raw;
    struct {
      bool limit_1;
      bool limit_2;
      bool limit_3;
      bool limit_4;
      bool motor_driver_fault_1;
      bool motor_driver_fault_2;
    } bits;
  };
} GpioReading;

void read_gpio(GpioReading* reading) {
  assert(reading != NULL);
  reading->bits.limit_1 = digital_read(LIMIT_1);
  reading->bits.limit_2 = digital_read(LIMIT_2);
  reading->bits.limit_3 = digital_read(LIMIT_3);
  reading->bits.limit_4 = digital_read(LIMIT_4);
  reading->bits.motor_driver_fault_1 = digital_read(NFAULT_MD1);
  reading->bits.motor_driver_fault_2 = digital_read(NFAULT_MD2);
}

void ctrl_heating(bool state) {
  digital_write(HE_CTRL, state);
}

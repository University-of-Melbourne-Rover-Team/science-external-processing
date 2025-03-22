#pragma once

#include <ADS7828.h>
#include <Wire.h>
#include <assert.h>

ADS7828 adc;
void setup_i2c() {
  Wire.begin();
  Wire.setClock(400000LU);
  adc.begin(0);
}

#define PWR_CTRL_IS 0
#define HE_IS_1 1
#define PROPLMD1 3
#define PROPLMD2 4
#define TEMP_SENSE_A1 5
#define TEMP_SENSE_B1 6
#define MPROBE 7
typedef struct ads_reading {
  uint16_t power_ctrl_sense;
  uint16_t heating_element_sense;
  uint16_t motor_driver_in_1;
  uint16_t motor_driver_in_2;
  uint16_t temp_sense_a1;
  uint16_t temp_sense_b1;
  uint16_t moisture_probe;
} AdsReading;

void read_ads(AdsReading* reading) {
  assert(reading != NULL);
  reading->power_ctrl_sense = adc.read(PWR_CTRL_IS);
  reading->heating_element_sense = adc.read(HE_IS_1);
  reading->motor_driver_in_1 = adc.read(PROPLMD1);
  reading->motor_driver_in_2 = adc.read(PROPLMD2);
  reading->temp_sense_a1 = adc.read(TEMP_SENSE_A1);
  reading->temp_sense_b1 = adc.read(TEMP_SENSE_B1);
  // NOTE: not currently using the moisture probe
  reading->moisture_probe = adc.read(MPROBE);
}

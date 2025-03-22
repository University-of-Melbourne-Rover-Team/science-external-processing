#pragma once
#include "drv8245.h"
#include <assert.h>

// Note: This file assumes bitfields are packed LSB first 

typedef enum drv_off_mode {
    DRV_COAST,
    DRV_BRAKE
} drv_off_mode_t;

typedef struct drv_state {
  drv8245_t drv;
  hw_timer_t* pwm_timer;

  struct {
    float pwm_duty;
    float pwm_freq;
    drv_off_mode_t off_mode;

    int64_t pwm_on_time_us;
    int64_t pwm_off_time_us;

    bool next_isr_is_period_start;
  } volatile;
} drv_state_t;

static void drv_set_spi_pins_with_fault_clear(drv8245_t *const drv, drv8245_spi_pins_t const * const pins) {
    drv8245_frameresult_blocking_t res = drv8245_set_SPI_IN_pins_blocking(drv, *pins);
    if (res.status.bits.FAULT) {
        if (res.status.bits.VMUV)
            drv8245_init_after_POR_blocking(drv);
        else
            drv8245_issue_CLR_FLT_blocking(drv);
    }
}

// Both outputs driven high
static drv8245_spi_pins_t const drv8245_pins_off_brake = {
    .en_in1  = false,
    .ph_in2  = false,
    .drvoff  = false,
    .drvoff2 = false
};

// Both outputs in Hi-Z state
static drv8245_spi_pins_t const drv8245_pins_off_coast = {
    .en_in1  = false,
    .ph_in2  = false,
    .drvoff  = true,
    .drvoff2 = false
};

// OUT1 driven high, OUT2 driven low
static drv8245_spi_pins_t const drv8245_pins_on_forward = {
    .en_in1  = true,
    .ph_in2  = true,
    .drvoff  = false,
    .drvoff2 = false
};

// OUT1 driven low, OUT2 driven high
const static drv8245_spi_pins_t drv8245_pins_on_backward = {
    .en_in1  = true,
    .ph_in2  = false,
    .drvoff  = false,
    .drvoff2 = false
};

//static int64_t pwm_timer_callback(alarm_id_t id, void *user_data);

void pwm_timer_callback(void* user_data);

void drv_set_mode_duty_freq(drv_state_t* self, drv_off_mode_t const off_mode, float const duty, float const freq) {
    assert(self != NULL);
    assert(-1 <= duty && duty <= 1);
    assert(freq > 0);

    float const period_us = 1.0f / freq * 1e6;

    noInterrupts();
    self->pwm_duty = duty;
    self->pwm_freq = freq;
    self->off_mode = off_mode;

    self->pwm_on_time_us = (int64_t)(period_us * fabsf(duty));
    self->pwm_off_time_us = (int64_t)(period_us - self->pwm_on_time_us);

    if (duty != 0 && self->pwm_timer == NULL) {
        // If duty is non-zero and timer not yet running then start the timer with an immediate callback
        self->next_isr_is_period_start = true;

        self->pwm_timer = timerBegin(1000000);
        timerAttachInterruptArg(self->pwm_timer, pwm_timer_callback, self);
        timerAlarm(self->pwm_timer, 0, true, 0);
        interrupts();
    } else if (duty == 0 && self->pwm_timer!= NULL) {
        // If duty is zero and timer is running then stop the timer
        timerEnd(self->pwm_timer);
        self->pwm_timer = NULL;
        interrupts();
        // Turn off the driver in the correct off mode (may be off already depending on stage of PWM period)
        drv_set_spi_pins_with_fault_clear(&self->drv,
            self->off_mode == DRV_COAST ? &drv8245_pins_off_coast : &drv8245_pins_off_brake);
    }
}

void drv_set_mode(drv_state_t* self, drv_off_mode_t const off_mode) {
    assert(self != NULL);
    drv_set_mode_duty_freq(self, off_mode, self->pwm_duty, self->pwm_freq);
}

void drv_set_pwm_duty(drv_state_t* self, float const duty) {
    assert(self != NULL);
    drv_set_mode_duty_freq(self, self->off_mode, duty, self->pwm_freq);
}

void drv_set_pwm_freq(drv_state_t* self, float const freq) {
    assert(self != NULL);
    drv_set_mode_duty_freq(self, self->off_mode, self->pwm_duty, freq);
}

void drv_set_duty_freq(drv_state_t* self, float const duty, float const freq) {
    assert(self != NULL);
    drv_set_mode_duty_freq(self, self->off_mode, duty, freq);
}

void pwm_timer_callback(void* user_data) {
    drv_state_t *self = (drv_state_t*) user_data;

    const bool this_isr_is_period_start = self->next_isr_is_period_start;
    self->next_isr_is_period_start ^= 1;

    drv8245_spi_pins_t pins = drv8245_pins_off_coast;
    if (this_isr_is_period_start) {
        // Start of PWM period, turn on the driver in the correct direction
        pins = self->pwm_duty >= 0 ? drv8245_pins_on_forward : drv8245_pins_on_backward;
    } else {
        // End of PWM period, turn off the driver in the correct off mode
        switch (self->off_mode) {
            case DRV_COAST:
                pins = drv8245_pins_off_coast;
                break;
            case DRV_BRAKE:
                pins = drv8245_pins_off_brake;
                break;
            default:
                assert(false);
        }
    }
    drv_set_spi_pins_with_fault_clear(&self->drv, &pins);

    // Even if not doing PWM still continue to schedule the next alarm to check for faults
    if (self->pwm_off_time_us <= 0) {
        self->next_isr_is_period_start = true;
        timerWrite(self->pwm_timer, self->pwm_on_time_us);
    }
    // Offset of next alarm is from when this was scheduled
    const int64_t offset = -(this_isr_is_period_start ? self->pwm_on_time_us : self->pwm_off_time_us);
    timerWrite(self->pwm_timer, offset);
}

void setup_drv(drv8245_t* self, uint chip_select);
void init_drv(drv8245_t* self);

drv_state_t drv_1;
drv_state_t drv_2;

#define SPI_CS1 D2
#define SPI_CS2 D3
#define SPI_CLK SCK
#define SPI_SDO MOSI
#define SPI_SDI MISO
void setup_spi() {
  setup_drv(&drv_1.drv, SPI_CS1);
  setup_drv(&drv_2.drv, SPI_CS2);

  init_drv(&drv_1.drv);
  init_drv(&drv_2.drv);
}

void setup_drv(drv8245_t* self, uint chip_select) {
  pin_mode(chip_select, OUTPUT);
  digital_write(chip_select, HIGH);

  drv8245_set_spi_inst(self);
  drv8245_set_nscs_pin(self, chip_select);
}

void init_drv(drv8245_t* self) {
  drv8245_config_t config;
  config.s_mode = DRV8245_S_MODE_PH_EN;
  config.en_in1_mode = DRV8245_PINREG_SEL_OR;
  config.ph_in2_mode = DRV8245_PINREG_SEL_OR;
  config.drvoff_mode = DRV8245_PINREG_SEL_OR;

  if (!drv8245_connected_device_id_matches(self, DRV8245_DEV_ID_DRV8245S_Q1)) {
    printf("uh oh");
  }

  drv8245_init_after_POR_blocking(self);

  drv8245_apply_configuration_blocking(self, config);
  drv8245_set_SPI_IN_lock_mode_blocking(self, false);
}

#define MOTOR_PWM_FREQ 500.f
void drv_stop_motor(uint8_t idx) {
  if (idx == 0) {
    drv_set_mode_duty_freq(&drv_1, DRV_COAST, 0, MOTOR_PWM_FREQ);
  } else if (idx == 1) {
    drv_set_mode_duty_freq(&drv_2, DRV_COAST, 0, MOTOR_PWM_FREQ);
  } else {
    assert(false);
  }
}

void drv_set_motor_velocity(uint8_t idx, float duty) {
  if (idx == 0) {
    drv_set_mode_duty_freq(&drv_1, DRV_COAST, duty, MOTOR_PWM_FREQ);
  } else if (idx == 1) {
    drv_set_mode_duty_freq(&drv_2, DRV_COAST, duty, MOTOR_PWM_FREQ);
  } else {
    assert(false);
  }
}

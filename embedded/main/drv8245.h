#pragma once

#include <stdint.h>
#include <limits.h>
#include <SPI.h>
#include "esp_system.h"

#define DRV8245_NSCS_HARDWARE UINT_MAX

typedef enum DRV8245_RegAccess {
    DRV8245_REGACCESS_READ,
    DRV8245_REGACCESS_WRITE,
    DRV8245_REGACCESS_READWRITE
} DRV8245_RegAccess_t;

// Read registers
#define DRV8245_REGADDR_DEVICE_ID     0x00
#define DRV8245_REGADDR_FAULT_SUMMARY 0x01
#define DRV8245_REGADDR_STATUS1       0x02
#define DRV8245_REGADDR_STATUS2       0x03
// Read and write registers
#define DRV8245_REGADDR_COMMAND       0x08
#define DRV8245_REGADDR_SPI_IN        0x09
#define DRV8245_REGADDR_CONFIG1       0x0A
#define DRV8245_REGADDR_CONFIG2       0x0B
#define DRV8245_REGADDR_CONFIG3       0x0C
#define DRV8245_REGADDR_CONFIG4       0x0D

#define DRV8245_DEV_ID_DRV8243S_Q1 0x32
#define DRV8245_DEV_ID_DRV8244S_Q1 0x42
#define DRV8245_DEV_ID_DRV8245S_Q1 0x52
#define DRV8245_DEV_ID_DRV8243P_Q1 0x36
#define DRV8245_DEV_ID_DRV8244P_Q1 0x46
#define DRV8245_DEV_ID_DRV8245P_Q1 0x56

#define DRV8245_PINREG_SEL_OR  0
#define DRV8245_PINREG_SEL_AND 1

#define TOCP_6000ns 0b00
#define TOCP_3000ns 0b01
#define TOCP_1500ns 0b10
#define TOCP_200ns  0b11

#define DRV8245_OCP_SEL_100  0b00
#define DRV8245_OCP_SEL_50_1 0b01
#define DRV8245_OCP_SEL_50_2 0b11
#define DRV8245_OCP_SEL_75   0b10

#define DRV8245_S_MODE_PH_EN       0b00 // Full-bridge mode where EN/IN1 if PWM input, PH/IN2 is direction input
#define DRV8245_S_MODE_Independent 0b01 // Independent control for 2 half-bridges
#define DRV8245_S_MODE_PWM1        0b10 // Full-bridge mode where EN/IN1 and PH/IN2 control PWM of each direction
#define DRV8245_S_MODE_PWM2        0b11

#define DRV8245_TOFF_20us 0b00
#define DRV8245_TOFF_30us 0b01
#define DRV8245_TOFF_40us 0b10
#define DRV8245_TOFF_50us 0b11

#define DRV8245_REG_LOCK_Lock      0b10  // Lock the CONFIG registers
#define DRV8245_REG_LOCK_Unlock1   0b00  // Unlock the CONFIG registers
#define DRV8245_REG_LOCK_Unlock2   0b01  // Unlock the CONFIG registers
#define DRV8245_REG_LOCK_Unlock3   0b11  // Unlock the CONFIG registers

#define DRV8245_SPI_IN_LOCK_Unlock 0b10  // Unlock the SPI_IN register
#define DRV8245_SPI_IN_LOCK_Lock1  0b00  // Lock the SPI_IN register
#define DRV8245_SPI_IN_LOCK_Lock2  0b01  // Lock the SPI_IN register
#define DRV8245_SPI_IN_LOCK_Lock3  0b11  // Lock the SPI_IN register

#define DRV8245_VMOV_SEL_VM_GreaterThan_35 0b00
#define DRV8245_VMOV_SEL_VM_GreaterThan_28 0b01
#define DRV8245_VMOV_SEL_VM_GreaterThan_18 0b10
#define DRV8245_VMOV_SEL_VMOV_Disabled     0b11

/* DEVICE_ID */
#define DRV8245_Reg_DEVICE_ID_Access DRV8245_REGACCESS_READ
typedef struct DRV8245_Reg_DEVICE_ID {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t REV_ID : 2; // Revision ID
            uint8_t DEV_ID : 6; // Device ID
        } bits;
    };
} DRV8245_Reg_DEVICE_ID_t;

/* FAULT_SUMMARY */
#define DRV8245_Reg_FAULT_SUMMARY_Access DRV8245_REGACCESS_READ
typedef struct DRV8245_Reg_FAULT_SUMMARY {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t OLA : 1;     // Open load condition in the ACTIVE state
            uint8_t TSD : 1;     // Over temperature
            uint8_t OCP : 1;     // Over current in one or more power FETs
            uint8_t VMUV : 1;    // VM under voltage
            uint8_t VMOV : 1;    // VM over voltage
            uint8_t FAULT : 1;   // Logic OR of SPI_ERR, POR, VMOV, VMUV, OCP, TSD
            uint8_t POR : 1;     // Power-on-reset
            uint8_t SPI_ERR : 1; // SPI communication fault in the previous SPI frame
        } bits;
    };
} DRV8245_Reg_FAULT_SUMMARY_t;

/* STATUS1 */
#define DRV8245_Reg_STATUS1_Access DRV8245_REGACCESS_READ

typedef struct DRV8245_Reg_STATUS1 {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t OCP_L2 : 1;    // Over current on the low-side FET on OUT2
            uint8_t OCP_H2 : 1;    // Over current on the high-side FET on OUT2
            uint8_t OCP_L1 : 1;    // Over current on the low-side FET on OUT1
            uint8_t OCP_H1 : 1;    // Over current on the high-side FET on OUT1
            uint8_t ACTIVE : 1;    // Device is in the ACTIVE state
            uint8_t ITRIP_CMP : 1; // Load current has reached the ITRIP regulation level
            uint8_t OLA2 : 1;      // Open load condition in the ACTIVE state on OUT2
            uint8_t OLA1 : 1;      // Open load condition in the ACTIVE state on OUT1
        } bits;
    };
} DRV8245_Reg_STATUS1_t;

/* STATUS2 */
#define DRV8245_Reg_STATUS2_Access DRV8245_REGACCESS_READ

typedef struct DRV8245_Reg_STATUS2 {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t OLP_CMP : 1;     // Output of the off-state diagnostics (OLP) comparator
            uint8_t RESERVED_A : 3;
            uint8_t ACTIVE : 1;      // Device is in the ACTIVE state
            uint8_t RESERVED_B : 2;
            uint8_t DRVOFF_STAT : 1; // Status of the DRVOFF pin
        } bits;
    };
} DRV8245_Reg_STATUS2_t;

/* COMMAND */
#define DRV8245_Reg_COMMAND_Access DRV8245_REGACCESS_READWRITE

typedef struct DRV8245_Reg_COMMAND {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t REG_LOCK : 2;     // Lock the CONFIG registers
            uint8_t RESERVED_A: 1;
            uint8_t SPI_IN_LOCK : 2;  // Lock the SPI_IN register
            uint8_t RESERVED_B: 2;
            uint8_t CLR_FLT : 1;      // Clear all faults reported in fault registers and de-assert nFAULT pin
        } bits;
    };
} DRV8245_Reg_COMMAND_t;

/* SPI_IN */
#define DRV8245_Reg_SPI_IN_Access DRV8245_REGACCESS_READWRITE

typedef struct DRV8245_Reg_SPI_IN {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t S_PH_IN2 : 1;   // Register bit equivalent of PH/IN2 pin when SPI_IN is unlocked
            uint8_t S_EN_IN1 : 1;   // Register bit equivalent of EN/IN1 pin when SPI_IN is unlocked
            uint8_t S_DRVOFF2 : 1;  // Register bit to shut off half-bridge 2 in independent mode when SPI_IN is unlocked
            uint8_t S_DRVOFF : 1;   // Register bit equivalent of DRVOFF pin when SPI_IN is unlocked
            uint8_t RESERVED : 4;
        } bits;
    };
} DRV8245_Reg_SPI_IN_t;

/* CONFIG1 */
#define DRV8245_Reg_CONFIG1_Access DRV8245_REGACCESS_READWRITE

typedef struct DRV8245_Reg_CONFIG1 {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t OLA_RETRY : 1;  // Fault reaction to retry setting on detection of open load during active (fault latched otherwise)
            uint8_t VMOV_RETRY : 1; // Fault reaction to retry setting on detection of VMOV (fault latched otherwise)
            uint8_t TSD_RETRY : 1;  // Fault reaction to retry setting on detection of over temperature (fault latched otherwise)
            uint8_t OCP_RETRY : 1;  // Fault reaction to retry setting on detection of over current (fault latched otherwise)
            uint8_t SSC_DIS : 1;    // Disable spread spectrum clocking feature
            uint8_t VMOV_SEL : 2;   // Thresholds for VM over voltage diagnostics
            uint8_t EN_OLA : 1;     // Enable open load detection in the active state
        } bits;
    };
} DRV8245_Reg_CONFIG1_t;

/* CONFIG2 */
#define DRV8245_Reg_CONFIG2_Access DRV8245_REGACCESS_READWRITE

typedef struct DRV8245_Reg_CONFIG2 {
    union {
        uint8_t raw_bye;
        struct {
            uint8_t S_ITRIP : 3;    // ITRIP level configuration
            uint8_t RESERVED: 2; // todo check
            uint8_t S_DIAG : 2;     // Load type indication
            uint8_t PWM_EXTEND : 1; // Access additional Hi-Z coast states in the PWM mode
        } bits;
    };
} DRV8245_Reg_CONFIG2_t;

/* CONFIG3 */
#define DRV8245_Reg_CONFIG3_Access DRV8245_REGACCESS_READWRITE

typedef struct DRV8245_Reg_CONFIG3 {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t S_MODE : 2;     // Device mode configuration
            uint8_t S_SR : 3;       // Slew rate configuration
            uint8_t RESERVED : 1;   // todo CHECK
            uint8_t TOFF : 2;       // TOFF time used for ITRIP current regulation
        } bits;
    };
} DRV8245_Reg_CONFIG3_t;

/* CONFIG4 */
#define DRV8245_Reg_CONFIG4_Access DRV8245_REGACCESS_READWRITE

typedef struct DRV8245_Reg_CONFIG4 {
    union {
        uint8_t raw_byte;
        struct {
            uint8_t PH_IN2_SEL : 1; // PH_IN2 pin/register logic combination when SPI_IN is unlocked
            uint8_t EN_IN1_SEL : 1; // EN_IN1 pin/register logic combination when SPI_IN is unlocked
            uint8_t DRVOFF_SEL : 1; // DRVOFF pin/register logic combination when SPI_IN is unlocked
            uint8_t OCP_SEL : 2;    // Threshold for over current detection configuration
            uint8_t RESERVED: 1; //TODO CHECK
            uint8_t TOCP_SEL : 2;   // Filter time for over current detection configuration
        } bits;
    };
    
} DRV8245_Reg_CONFIG4_t;

/* Interface typedefs */

typedef struct drv8245_frameresult_blocking {
    // Status bits from FAULT_SUMMARY register
    union {
        uint8_t raw_byte;
        struct {
            uint8_t SPI_ERR : 1;
            uint8_t TSD : 1;
            uint8_t OCP : 1;
            uint8_t VMUV : 1;
            uint8_t VMOV : 1;
            uint8_t FAULT : 1;
            uint8_t RESERVED : 2;
        } bits;
    } status;
    uint8_t report;
} drv8245_frameresult_blocking_t;

typedef enum drv8245_frameresult_nonblocking {
    DRV8245_FRAMERESULT_NONBLOCKING_SUCCESS,        // Successfully sent command
    DRV8245_FRAMERESULT_NONBLOCKING_SPI_FIFO_FULL   // Could not send command since SPI FIFO transmit buffer is full
} drv8245_frameresult_nonblocking_t;

typedef struct drv8245_config {
    uint8_t s_mode;
    uint8_t en_in1_mode, ph_in2_mode, drvoff_mode;
} drv8245_config_t;

typedef struct drv8245_spi_pins {
    uint8_t en_in1;
    uint8_t ph_in2;
    uint8_t drvoff;
    uint8_t drvoff2;
} drv8245_spi_pins_t;

typedef struct drv8245 {
    //spi_inst_t *spi;
    uint gpio_nscs;
} drv8245_t;

/* Register interface */

#define DATASHEET_T_READY_US 1000

drv8245_frameresult_blocking_t drv8245_do_frame_transaction_blocking(drv8245_t const *const drv, uint8_t const reg_addr, uint8_t const write, uint8_t const write_data);

static inline bool using_software_nscs(drv8245_t const *const drv) {
    return drv->gpio_nscs != DRV8245_NSCS_HARDWARE;
}

void drv8245_set_spi_inst(drv8245_t *const drv) {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
}

uint drv8245_set_nscs_pin(drv8245_t *const drv, uint const gpio_pin) {
    drv->gpio_nscs = gpio_pin;
    if (using_software_nscs(drv)) {
        digital_write(gpio_pin, true);
    }
}

drv8245_frameresult_blocking_t drv8245_read_reg_blocking(drv8245_t const *const drv, uint8_t const reg_addr) {
    return drv8245_do_frame_transaction_blocking(drv, reg_addr, false, 0);
}

drv8245_frameresult_blocking_t drv8245_write_reg_blocking(drv8245_t const *const drv, uint8_t const reg_addr, uint8_t const data) {
    return drv8245_do_frame_transaction_blocking(drv, reg_addr, true, data);
}

/*
drv8245_frameresult_nonblocking_t drv8245_write_reg_nonblocking(drv8245_t const *const drv, uint8_t const reg_addr, uint8_t const data)
{
    return drv8245_do_frame_transaction_nonblocking(drv, reg_addr, data);
}
*/


drv8245_frameresult_blocking_t drv8245_update_reg_bits_blocking(drv8245_t const *const drv, uint8_t const reg_addr, uint8_t const mask, uint8_t const val) {
    drv8245_frameresult_blocking_t r = drv8245_write_reg_blocking(drv, reg_addr, val);
    uint8_t intended_val = (r.report & ~mask) | val;
    // If intended value not equal to value written, rewrite
    if (intended_val != val) {
        return drv8245_write_reg_blocking(drv, reg_addr, intended_val);
    }
    return r;
}

drv8245_frameresult_blocking_t drv8245_issue_CLR_FLT_blocking(drv8245_t const *const drv) {
    DRV8245_Reg_COMMAND_t reg_cmd = { .raw_byte = 0 };
    reg_cmd.bits.CLR_FLT = 1;
    return drv8245_update_reg_bits_blocking(drv, DRV8245_REGADDR_COMMAND, reg_cmd.raw_byte, reg_cmd.raw_byte);
}

drv8245_frameresult_blocking_t drv8245_init_after_POR_blocking(drv8245_t const *const drv) {
    sleep_us(DATASHEET_T_READY_US);
    return drv8245_issue_CLR_FLT_blocking(drv);
}

uint8_t drv8245_connected_device_id_matches(drv8245_t const *const drv, uint8_t const id) {
    drv8245_read_reg_blocking(drv, DRV8245_REGADDR_DEVICE_ID).report == id;
}

void drv8245_apply_configuration_blocking(drv8245_t const *const drv, drv8245_config_t const config) {
    /*
     * Steps:
     * 1. Unlock config registers
     * 2. Apply configuration
     * 3. Lock config registers
     */

    // TODO: Handle possibility of res.status.bits.SPI_ERR - SPI error on previous transaction
    // This is why we keep reassigning res without using the value.

    // Unlock config registers and store current value of SPI_IN_LOCK (in reg_cmd.raw_byte)
    // Note this will lock the SPI_IN register
    DRV8245_Reg_COMMAND_t reg_cmd = { .raw_byte = 0 };
    reg_cmd.bits.REG_LOCK = DRV8245_REG_LOCK_Unlock1;

    drv8245_frameresult_blocking_t res;
    res = drv8245_write_reg_blocking(drv, DRV8245_REGADDR_COMMAND, reg_cmd.raw_byte);
    reg_cmd.raw_byte = res.report;

    // Apply mode select configuration
    DRV8245_Reg_CONFIG3_t reg_config3 = { .raw_byte = 0 };
    reg_config3.bits.S_MODE = 0x3;
    uint8_t const s_mode_mask = reg_config3.raw_byte;
    reg_config3.bits.S_MODE = config.s_mode;
    
    res = drv8245_update_reg_bits_blocking(drv, DRV8245_REGADDR_CONFIG3, s_mode_mask, reg_config3.raw_byte);
    
    // Apply pin mode configuration
    DRV8245_Reg_CONFIG4_t reg_config4 = { .raw_byte = 0 };
    reg_config4.bits.EN_IN1_SEL = true;
    reg_config4.bits.PH_IN2_SEL = true;
    reg_config4.bits.DRVOFF_SEL = true;
    uint8_t const pinreg_sel_mask = reg_config4.raw_byte;
    reg_config4.bits.EN_IN1_SEL = config.en_in1_mode;
    reg_config4.bits.PH_IN2_SEL = config.ph_in2_mode;
    reg_config4.bits.DRVOFF_SEL = config.drvoff_mode;
    res = drv8245_update_reg_bits_blocking(drv, DRV8245_REGADDR_CONFIG4, pinreg_sel_mask, reg_config4.raw_byte);

    // Lock config registers and restore previous value of SPI_IN_LOCK
    reg_cmd.bits.REG_LOCK = DRV8245_REG_LOCK_Lock;
    res = drv8245_write_reg_blocking(drv, DRV8245_REGADDR_COMMAND, reg_cmd.raw_byte);
}

drv8245_frameresult_blocking_t drv8245_set_SPI_IN_lock_mode_blocking(drv8245_t const *const drv, uint8_t const locked) {
    DRV8245_Reg_COMMAND_t reg_cmd = { .raw_byte = 0 };
    reg_cmd.bits.SPI_IN_LOCK = 0x3;
    uint8_t const spi_in_mask = reg_cmd.raw_byte;
    reg_cmd.bits.SPI_IN_LOCK = locked ? DRV8245_SPI_IN_LOCK_Lock1 : DRV8245_SPI_IN_LOCK_Unlock;
    return drv8245_update_reg_bits_blocking(drv, DRV8245_REGADDR_COMMAND, spi_in_mask, reg_cmd.raw_byte);
}

uint8_t drv8245_get_SPI_IN_is_locked_blocking(drv8245_t const *const drv) {
    DRV8245_Reg_COMMAND_t reg_cmd;
    reg_cmd.raw_byte = drv8245_read_reg_blocking(drv, DRV8245_REGADDR_COMMAND).report;
    return reg_cmd.bits.SPI_IN_LOCK != DRV8245_SPI_IN_LOCK_Unlock;
}

drv8245_frameresult_blocking_t drv8245_set_SPI_IN_pins_blocking(drv8245_t const *const drv, drv8245_spi_pins_t const pins) {
    DRV8245_Reg_SPI_IN_t reg_spi = { .raw_byte = 0 };
    reg_spi.bits.S_EN_IN1 = pins.en_in1;
    reg_spi.bits.S_PH_IN2 = pins.ph_in2;
    reg_spi.bits.S_DRVOFF = pins.drvoff;
    reg_spi.bits.S_DRVOFF2 = pins.drvoff2;
    return drv8245_write_reg_blocking(drv, DRV8245_REGADDR_SPI_IN, reg_spi.raw_byte);
}

  /*
drv8245_frameresult_nonblocking_t drv8245_set_SPI_IN_pins_nonblocking(drv8245_t const *const drv, drv8245_spi_pins_t const pins)
{
    DRV8245_Reg_SPI_IN_t reg_spi = { .raw_byte = 0 };
    reg_spi.bits.S_EN_IN1 = pins.en_in1;
    reg_spi.bits.S_PH_IN2 = pins.ph_in2;
    reg_spi.bits.S_DRVOFF = pins.drvoff;
    reg_spi.bits.S_DRVOFF2 = pins.drvoff2;
    return drv8245_write_reg_nonblocking(drv, DRV8245_REGADDR_SPI_IN, reg_spi.raw_byte);
}
*/

drv8245_frameresult_blocking_t drv8245_do_frame_transaction_blocking(drv8245_t const *const drv, uint8_t const reg_addr, uint8_t const write, uint8_t const write_data) {
    // B15 = 0 -> standard frame, B14 = 1 -> read, then address
    uint16_t send =
        0b0 << 15 |
        !write << 14 |
        (uint8_t)reg_addr << 8 |
        write_data;
    uint16_t recv;
    // Data transferred MSB first for both master and slave
    // Transfer 1 half word (i.e. 16 bits)
    if (using_software_nscs(drv)) {
        digital_write(drv->gpio_nscs, false);
    }

    recv = SPI.transfer16(send);

    if (using_software_nscs(drv)) {
        digital_write(drv->gpio_nscs, true);
    }

    drv8245_frameresult_blocking_t r;
    r.status.raw_byte = recv >> 8;
    r.report = recv & 0x00ff;
    return r;
}

/*
drv8245_frameresult_nonblocking_t drv8245_do_frame_transaction_nonblocking(drv8245_t const *const drv, uint8_t const reg_addr, uint8_t const write_data)
{
    if (!spi_is_writable(drv->spi))
        return DRV8245_FRAMERESULT_NONBLOCKING_SPI_FIFO_FULL;

    // Transmit data

    

    spi_get_hw(drv->spi)->dr =
        0b0 << 15 |
        0b0 << 14 | // Write operation, since not waiting for result for read
        (uint8_t)reg_addr << 8 |
        write_data;
    
    // Note that we do not read from the receive FIFO since we would have to wait for the transaction to complete - means the receive FIFO could potentially be full after this transaction
    return DRV8245_FRAMERESULT_NONBLOCKING_SUCCESS;
}
*/


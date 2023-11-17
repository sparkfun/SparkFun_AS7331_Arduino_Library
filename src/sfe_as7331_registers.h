/*
  SPDX-License-Identifier: MIT

  Copyright (c) 2023 SparkFun Electronics

*/

#pragma once
#include <stdint.h>

/// I2C addresses
// 7-bit address defined as [1, 1, 1, 0, 1, A1, A0] where A1/A0 are the address pins tied high or low
#define AS7331_ADDR_DEFAULT     0x74
#define AS7331_ADDR_SEC         0x75
#define AS7331_ADDR_TER         0x76
#define AS7331_ADDR_QUA         0x77

#define AS7331_DEFAULT_DEV_ID   0x21
#define AS7331_DEV_ID_HIGH      0x2


/// Enum Settings

typedef enum {
    // 0b00X invalid
    DEVICE_MODE_CFG = 0x2,
    DEVICE_MODE_MEAS = 0x3
    // 0b1XX invalid
} as7331_device_op_state_t;

typedef enum {
    POWER_DOWN_DISABLE = 0x0,
    POWER_DOWN_ENABLE
} as7331_power_state_t;

typedef enum {
    START_STATE_DISABLED = 0x0,
    START_STATE_ENABLED
} as7331_startstate_t;

typedef enum {
    GAIN_2048 = 0x0000,
    GAIN_1024,
    GAIN_512,
    GAIN_256,
    GAIN_128,
    GAIN_64,
    GAIN_32,
    GAIN_16,
    GAIN_8,
    GAIN_4,
    GAIN_2,
    GAIN_1
} as7331_gain_t;

typedef enum {
    SYN_TEMP_DISABLED = 0x0,
    SYN_TEMP_ENABLED
} as7331_ext_syn_temp_meas_t;

typedef enum {
    CCLK_1_024_MHZ = 0x00,
    CCLK_2_048_MHZ,
    CCLK_4_096_MHZ,
    CCLK_8_192_MHZ
} as7331_conv_clk_freq_t;

typedef enum {
    READYPIN_PUSHPULL = 0x0,
    READYPIN_OPENDRAIN
} as7331_ready_pin_mode_t;

typedef enum {
    STANDBY_DISABLED = 0x0,
    STANDBY_ENABLED
} as7331_standby_mode_t;

typedef enum {
    MEAS_MODE_CONT = 0x0,
    MEAS_MODE_CMD,
    MEAS_MODE_SYNS,
    MEAS_MODE_SYND
} as7331_meas_mode_t;

typedef enum {
    INDEX_NO_REPEAT_START = 0x0,
    INDEX_REPEAT_START
} as7331_simple_reg_read_mode_t;

typedef enum {
    TIME_1MS = 0x0,
    TIME_2MS,
    TIME_4MS,
    TIME_8MS,
    TIME_16MS,
    TIME_32MS,
    TIME_64MS,
    TIME_128MS,
    TIME_256MS,
    TIME_512MS,
    TIME_1024MS,
    TIME_2048MS,
    TIME_4096MS,
    TIME_8192MS,
    TIME_16384MS
} as7331_conv_time_t;

typedef enum {
    DIVIDER_DISABLED = 0x0,
    DIVIDER_ENABLED = 0x1
} as7331_divider_enable_t;

typedef enum {
    DIV_2 = 0x0,
    DIV_4,
    DIV_8,
    DIV_16,
    DIV_32,
    DIV_64,
    DIV_128,
    DIV_256
} as7331_divider_val_t;

/// Configuration Registers
// These registers can only be accessed in the configuration state, and they
// are all 8 bits long.

#define SFE_AS7331_REGISTER_CFG_OSR 0x00
typedef union {
    struct {
        uint8_t dos         : 3;
        uint8_t sw_res      : 1;
        uint8_t reserved    : 2;
        uint8_t pd          : 1;
        uint8_t ss          : 1;
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_osr_t;

#define SFE_AS7331_REGISTER_CFG_AGEN  0x02
typedef union {
    struct {
        uint8_t mut         : 4;
        uint8_t devid       : 4;
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_agen_t;

#define SFE_AS7331_REGISTER_CFG_CREG1 0x06
typedef union {
    struct {
        uint8_t time        : 4;
        uint8_t gain        : 4;
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_creg1_t;

#define SFE_AS7331_REGISTER_CFG_CREG2 0x07
typedef union {
    struct {
        uint8_t div         : 3;
        uint8_t en_div      : 1;
        uint8_t reserved    : 2;
        uint8_t en_tm       : 1;
        uint8_t reserved1   : 1;
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_creg2_t;

#define SFE_AS7331_REGISTER_CFG_CREG3 0x08
typedef union {
    struct {
        uint8_t cclk        : 2;
        uint8_t reserved    : 1;
        uint8_t rdyod       : 1;
        uint8_t sb          : 1;
        uint8_t reserved1   : 1;
        uint8_t mmode       : 2;
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_creg3_t;

#define SFE_AS7331_REGISTER_CFG_BREAK 0x09
typedef uint8_t sfe_as7331_reg_cfg_break_t;

#define SFE_AS7331_REGISTER_CFG_EDGES 0x0A
typedef uint8_t sfe_as7331_reg_cfg_edges_t;

#define SFE_AS7331_REGISTER_CFG_OPTREG 0x0B
typedef union {
    struct {
        uint8_t init_idx    : 1;
        uint8_t reserved    : 7;
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_optreg_t;


/// Measurement Registers
// These registers can only be accessed in the measurement state. They are
// read-only and 16 bits long, except OUTCONV, which is 24 bits long.

#define SFE_AS7331_REGISTER_MEAS_OSR_STATUS 0x00
typedef union {
    struct {
        sfe_as7331_reg_cfg_osr_t osr;
        uint8_t powerstate      : 1;
        uint8_t standbystate    : 1;
        uint8_t notready        : 1;
        uint8_t ndata           : 1;
        uint8_t ldata           : 1;
        uint8_t adcof           : 1;
        uint8_t mresof          : 1;
        uint8_t outconvof       : 1;
    };
    uint16_t word;
} sfe_as7331_reg_meas_osr_status_t;

#define SFE_AS7331_REGISTER_MEAS_TEMP 0x01
typedef union {
    struct {
        uint16_t temp           : 12;
        uint8_t zeros           : 4;
    };
    uint16_t word;
} sfe_as7331_reg_meas_temp_t;

#define SFE_AS7331_REGISTER_MEAS_MRES1 0x02
typedef union {
    struct {
        uint8_t res_l;
        uint8_t res_h;
    };
    uint16_t word;
} sfe_as7331_reg_meas_mres1_t;

#define SFE_AS7331_REGISTER_MEAS_MRES2 0x03
typedef union {
    struct {
        uint8_t res_l;
        uint8_t res_h;
    };
    uint16_t word;
} sfe_as7331_reg_meas_mres2_t;


#define SFE_AS7331_REGISTER_MEAS_MRES3 0x04
typedef union {
    struct {
        uint8_t res_l;
        uint8_t res_h;
    };
    uint16_t word;
} sfe_as7331_reg_meas_mres3_t;


#define SFE_AS7331_REGISTER_MEAS_OUTCONV_L 0x05
typedef union {
    struct {
        uint8_t outconv_l;
        uint8_t outconv_m;
    };
    uint16_t word;
} sfe_as7331_reg_meas_outconv_l_t;


#define SFE_AS7331_REGISTER_MEAS_OUTCONV_H 0x06
typedef union {
    struct {
        uint8_t outconv_h;
        uint8_t zeros;
    };
    uint16_t word;
} sfe_as7331_reg_meas_outconv_h_t;

/*
    SparkFun Spectral UV Sensor - AS7331

    Qwiic 1x1
    https://www.sparkfun.com/products/23517
    Qwiic Mini
    https://www.sparkfun.com/products/23518

    Repository
    https://github.com/sparkfun/SparkFun_AS7331_Arduino_Library

    SPDX-License-Identifier: MIT

    Copyright (c) 2023 SparkFun Electronics

    Name: sfeAS7331.h

    Description:
    SfeAS7331Driver is a comms-agnostic driver for the AS7331 Spectral UV
    sensor that uses the SparkFun Toolkit. The SfeAS7331ArdI2C class defines
    the Arduino specific behavior for initializing and interadcting with devices.

*/

#pragma once

#include <stdint.h>
#include <SparkFun_Toolkit.h>

///////////////////////////////////////////////////////////////////////////////
// I2C Addressing
///////////////////////////////////////////////////////////////////////////////
// 7-bit address defined as [1, 1, 1, 0, 1, A1, A0] where A1/A0 are the
// physical address pins tied high or low
const uint8_t kDefaultAS7331Addr = 0x74;    // A1 = 0, A0 = 0
const uint8_t kSecondaryAS7331Addr = 0x75;  // A1 = 0, A1 = 1
const uint8_t kTertiaryAS7331Addr = 0x76;   // A1 = 1, A0 = 0
const uint8_t kQuaternaryAS7331Addr = 0x77; // A1 = 1, A0 = 1

const uint8_t kDefaultAS7331DeviceID = 0x21;  // When polling the AGEN register, this should be returned on boot.
const uint8_t kAS7331DeviceIDTopNibble = 0x2; // Top nibble of the AGEN byte, always the same.

///////////////////////////////////////////////////////////////////////////////
// Enum Definitions
///////////////////////////////////////////////////////////////////////////////

// Device Operating Mode
typedef enum
{
    // 0b00X invalid
    DEVICE_MODE_CFG = 0x2,
    DEVICE_MODE_MEAS = 0x3
    // 0b1XX invalid
} as7331_dev_op_state_t;

// Sensor gain, able to be read as (1 << (11 - gain)) aka 2^(11-gain).
typedef enum
{
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

// Conversion Clock Frequency available settings, read as (1024 * (1 << cclk))
// aka 1024*2^cclk.
typedef enum
{
    CCLK_1_024_MHZ = 0x00,
    CCLK_2_048_MHZ,
    CCLK_4_096_MHZ,
    CCLK_8_192_MHZ
} as7331_conv_clk_freq_t;

// Measurement Mode types
typedef enum
{
    MEAS_MODE_CONT = 0x0, // Continuous mode
    MEAS_MODE_CMD,        // Command/OneShot mode
    MEAS_MODE_SYNS,       // SYNchronous Start mode
    MEAS_MODE_SYND        // SYNchronous start and enD mode
} as7331_meas_mode_t;

// Time conversion values in CONT, CMD, SYNS modes. Read as (1 << tconv) aka
// 2^tconv.
typedef enum
{
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

// Predivider values. Read as (1 << (1+div)) aka 2^(1+div).
typedef enum
{
    DIV_2 = 0x0,
    DIV_4,
    DIV_8,
    DIV_16,
    DIV_32,
    DIV_64,
    DIV_128,
    DIV_256
} as7331_divider_val_t;

// Type of UV to pick.
typedef enum
{
    AS7331_UVA,
    AS7331_UVB,
    AS7331_UVC
} as7331_uv_type;

///////////////////////////////////////////////////////////////////////////////
// Configuration Register Descriptions
//
// Can only be accessed in the Configuration operation state. All 8 bits long.
//
// The following registers map to the underlying registers of the device.
///////////////////////////////////////////////////////////////////////////////

const uint8_t kSfeAS7331RegCfgOsr = 0x00; // Register address

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union {
    struct
    {
        uint8_t dos : 3;      // Device Operating State   - OSR[2:0]
        uint8_t sw_res : 1;   // Software Reset           - OSR[3]
        uint8_t reserved : 2; // Reserved, don't write.   - OSR[4:5]
        uint8_t pd : 1;       // Power Down Enabled       - OSR[6]
        uint8_t ss : 1;       // Start State              - OSR[7]
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_osr_t;

const uint8_t kSfeAS7331RegCfgAgen = 0x02; // Register address

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union {
    struct
    {
        uint8_t mut : 4;   // Increments when changes are made to the control
                           // registers. Defaults to 0b0001 aka 0x1.
        uint8_t devid : 4; // Always equals 0b0010 aka 0x2.
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_agen_t;

const uint8_t kSfeAS7331RegCfgCreg1 = 0x06; // Register address

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union {
    struct
    {
        uint8_t time : 4; // Conversion time  - CREG1[3:0]
        uint8_t gain : 4; // Sensor gain      - CREG1[7:4]
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_creg1_t;

const uint8_t kSfeAS7331RegCfgCreg2 = 0x07; // Register address

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union {
    struct
    {
        uint8_t div : 3;       // Digital Divider value                            - CREG2[2:0]
        uint8_t en_div : 1;    // Whether the digital divider is enabled           - CREG2[3]
        uint8_t reserved : 2;  // Reserved, don't write.                           - CREG2[5:4]
        uint8_t en_tm : 1;     // Whether temperature is calculated in SYND mode.  - CREG2[6]
        uint8_t reserved1 : 1; // Reserved, don't write.                           - CREG2[7]
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_creg2_t;

const uint8_t kSfeAS7331RegCfgCreg3 = 0x08; // Register Address

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union {
    struct
    {
        uint8_t cclk : 2;      // Conversion clock selection.      - CREG3[1:0]
        uint8_t reserved : 1;  // Reserved, don't write.           - CREG3[2]
        uint8_t rdyod : 1;     // Output mode of the ready pin.    - CREG3[3]
        uint8_t sb : 1;        // Standby mode bit.                - CREG3[4]
        uint8_t reserved1 : 1; // Reserved, don't write.           - CREG3[5]
        uint8_t mmode : 2;     // Measurement mode selection.      - CREG3[7:6]
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_creg3_t;

const uint8_t kSfeAS7331RegCfgBreak = 0x09; // Register address, register is a single uint8_t.

const uint8_t kSfeAS7331RegCfgEdges = 0x0A; // Register address, register is a single uint8_t.

const uint8_t kSfeAS7331RegCfgOptReg = 0x0B; // Register address

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union {
    struct
    {
        uint8_t init_idx : 1; // I2C repeat start mode flag.  - OPTREG[0]
        uint8_t reserved : 7; // Reserved, don't write.       - OPTREG[7:1]
    };
    uint8_t byte;
} sfe_as7331_reg_cfg_optreg_t;

///////////////////////////////////////////////////////////////////////////////
// Measurement Register Descriptions
//
// Can only be accessed in the Measure operation state. All 16 bits long. They
// are all read-only except the bottom byte of the OSR/STATUS register.
//
// The following registers map to the underlying registers of the device.
///////////////////////////////////////////////////////////////////////////////

const uint8_t kSfeAS7331RegMeasOsrStatus = 0x00; // Register address

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union {
    struct
    {
        sfe_as7331_reg_cfg_osr_t osr; // See OSR configuration register above.        - OSRSTAT[7:0]
        uint8_t powerstate : 1;       // Power down state.                            - OSRSTAT[8]
        uint8_t standbystate : 1;     // Standby mode state.                          - OSRSTAT[9]
        uint8_t notready : 1;         // Inverted value of the ready pin.             - OSRSTAT[10]
        uint8_t ndata : 1;            // Indicates new data available.                - OSRSTAT[11]
        uint8_t ldata : 1;            // Indicates data overwrite prior to retrieval. - OSRSTAT[12]
        uint8_t adcof : 1;            // OVF of at least one ADC channel.             - OSRSTAT[13]
        uint8_t mresof : 1;           // OVF of at least one of the MRES registers.   - OSRSTAT[14]
        uint8_t outconvof : 1;        // OVF of the internal 24-bit time reference.   - OSRSTAT[15]
    };
    uint16_t word;
} sfe_as7331_reg_meas_osr_status_t;

// Output result registers. 16-bit values unless noted otherwise.
const uint8_t kSfeAS7331RegMeasTemp = 0x01; // 12-bit temperature, MS 4-bits 0.
const uint8_t kSfeAS7331RegMeasMres1 = 0x02;
const uint8_t kSfeAS7331RegMeasMres2 = 0x03;
const uint8_t kSfeAS7331RegMeasMres3 = 0x04;
const uint8_t kSfeAS7331RegMeasOutConvL = 0x05; // First 16-bits of 24-bit OUTCONV.
const uint8_t kSfeAS7331RegMeasOutConvH = 0x06; // LSB is MSB of OUTCONV, MSB is 0.

///////////////////////////////////////////////////////////////////////////////

class SfeAS7331Driver
{
  public:
    // Default initialization values based on the datasheet. See SfeAS7331Driver::setDefaultSettings for
    // an explanation of the values.
    SfeAS7331Driver()
        : _theBus{nullptr}, _breakTime{25}, _numEdges{1}, _readyPinMode{false},
          _dividerEnabled{false}, _tempConvEnabled{true}, _indexMode{true}, _standbyState{false}, _startState{false},
          _powerDownEnableState{true}, _opMode{DEVICE_MODE_CFG}, _sensorGain{GAIN_2}, _cclk{CCLK_1_024_MHZ},
          _mmode{MEAS_MODE_CMD}, _conversionTime{TIME_64MS}, _dividerRange{DIV_2}, _uva{0.0f}, _uvb{0.0f}, _uvc{0.0f},
          _temperature{0.0f}, _outputConversionTime{0U}, _conversionA{0.0f}, _conversionB{0.0f}, _conversionC{0.0f}
    {
    }

    /// @brief This method is called to initialize the AS7331 device through the
    /// specified bus.
    /// @param theBus Pointer to the bus object.
    /// @return True if successful, false if it fails.
    bool begin(sfeTkIBus *theBus = nullptr);

    /// @brief Requests the device ID from the sensor.
    /// @return The device ID of the sensor.
    uint8_t getDeviceID(void);

    /// @brief Sets the communication bus to the specified bus.
    /// @param theBus Bus to set as the communication devie.
    void setCommunicationBus(sfeTkIBus *theBus);

    /// @brief Helper class that sets up the sensor and state in the POR
    /// configuration.
    /// @param runSoftReset Flag that runs the soft reset function to reset the
    /// device.
    /// @return True if successful, false otherwise.
    bool runDefaultSetup(const bool &runSoftReset = false);

    /// @brief Puts the sensor in the specified measurement mode.
    /// @param measMode Measurement mode to enter.
    /// @param startMeasure Flag to start measuring immediately if set.
    /// @return True if successful, false otherwise.
    bool prepareMeasurement(const as7331_meas_mode_t &measMode = MEAS_MODE_CONT, const bool &startMeasure = false);

    /// @brief Performs a soft reset of the sensor.
    /// @return True if successful, false otherwise.
    bool reset(void);

    /// @brief Reads the sensor's temperature, converts it to a usable form, and
    /// saves it to the internal temperature variable.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t readTemp(void);

    /// @brief Reads the sensor's UVA register, converts it to a usable form, and
    /// saves it to the internal UVA variable.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t readUVA(void);

    /// @brief Reads the sensor's UVB register, converts it to a usable form, and
    /// saves it to the internal UVB variable.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t readUVB(void);

    /// @brief Reads the sensor's UVC register, converts it to a usable form, and
    /// saves it to the internal UVC variable.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t readUVC(void);

    /// @brief Read's all three UV registers, converts them to a usable form, then
    /// saves them to their respective internal variable.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t readAllUV(void);

    /// @brief Read the sensor's temperature, UV, and external time conversion
    /// clock counts, converts them, and then saves them to their respective
    /// internal variable.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t readAll(void);

    /// @brief Read the conversion clock counts register and saves it to the
    /// internal output conversion time variable.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t readOutConv(void);

    /// @brief Returns the last valid UVA reading.
    /// @return A float of the UVA reading.
    float getUVA(void);

    /// @brief Returns the last valid UVB reading.
    /// @return A float of the UVB reading.
    float getUVB(void);

    /// @brief Returns the last valid UVC reading.
    /// @return A float of the UVC reading.
    float getUVC(void);

    /// @brief Returns the last valid Temperature reading.
    /// @return A float of the Temperature reading.
    float getTemp(void);

    /// @brief Returns the last valid output conversion.
    /// @return The output conversion time in number of clock cycles.
    uint32_t getOutConv(void);

    /// @brief Getter for the currently configured gain.
    /// @return Sensor's gain expressed as (1 << (11 - gain)).
    as7331_gain_t getGainRaw(void);

    /// @brief Getter for the currently configured gain.
    /// @return Sensor's gain expressed as a unitless scalar.
    uint16_t getGainValue(void);

    /// @brief Sets the UV sensor's gain.
    /// @param gain The gain to set the sensor to.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setGain(const as7331_gain_t &gain);

    /// @brief Getter for the currently configured conversion clock.
    /// @return Sensor's conversion clock expressed as 1024*(1 << cclk).
    as7331_conv_clk_freq_t getCClkRaw(void);

    /// @brief Getter for the currently configured conversion clock.
    /// @return Sensor's conversion clock expressed in kHz.
    uint16_t getCClkKHz(void);

    /// @brief Set the sensor's internal clock speed.
    /// @param cclk Clock speed to set on the sensor.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setCClk(const as7331_conv_clk_freq_t &cclk);

    /// @brief Getter for the currently configured conversion time.
    /// @return Sensor's conversion time expressed as (1 << time).
    as7331_conv_time_t getConversionTimeRaw(void);

    /// @brief Getter for the currently configured conversion time.
    /// @return Sensor's conversion time in milliseconds.
    uint16_t getConversionTimeMillis(void);

    /// @brief Sets the conversion time that the sensor will run to.
    /// @param convTime Conversion time to set the sensor to.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setConversionTime(const as7331_conv_time_t &convTime);

    /// @brief Getter for the currently configured pin mode.
    /// @return False if push-pull, true if open-drain.
    bool getReadyPinMode(void);

    /// @brief Sets the ready pin type to push-pull or open-drain.
    /// @param pinMode Mode to set the ready pin to.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setReadyPinMode(const bool &pinMode);

    /// @brief Getter for the currently configured divider status.
    /// @return True if Internal predivider is enabled, false otherwise.
    bool getDigitalDividerEnabled(void);

    /// @brief Enables or disables the internal UV result divider.
    /// @param isEnabled Enable or disable the divder.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setDigitalDividerEnabled(const bool &isEnabled);

    /// @brief Getter for the currently configured divider range.
    /// @return Sensor's internal UV predivider range.
    as7331_divider_val_t getDigitalDividerRange(void);

    /// @brief Sets the value of the internal UV result divider.
    /// @param divider Divider value to set.
    /// @param setEnableDiv Option to turn on the divider if desired.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setDigitalDividerRange(const as7331_divider_val_t &divider, const bool &enableDiv = true);

    /// @brief Getter for the SYND temperature conversion status.
    /// @return True if temperature conversion is enabled in SYND mode.
    bool getSyndTempConversionEnabled(void);

    /// @brief Enables or disables temperature conversion when in SYND mode.
    /// @param isEnabled Enable or disable the feature.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setSyndTempConversionEnabled(const bool &isEnabled);

    /// @brief Getter for the currently configured I2C compatibility mode.
    /// @return True if the device will respond to repeat starts, false otherwise.
    bool getIndexMode(void);

    /// @brief Set the index mode for compatibility with I2C controllers that
    /// don't support repeated start.
    /// @param indexMode Simple or standard I2C addressing mode.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setIndexMode(const bool &indexMode);

    /// @brief Getter for the currently configured minimum break time in CONT,
    /// CMD, SYNS modes.
    /// @return Sensor's breaktime in 8us steps.
    uint8_t getBreakTime(void);

    /// @brief Set the minimum time between measurements in CONT, SYNS, SYND modes.
    /// @param breakTime Time between measurements, 8us step time, max 2048us. A 0
    /// value is a minimum of 3 cclk cycles.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setBreakTime(const uint8_t &breakTime);

    /// @brief Getter for the currently configured minimum number of edges to end
    /// conversion when in SYND mode.
    /// @return Sensor's minimum number of edges, minimum 1 edge.
    uint8_t getNumEdges(void);

    /// @brief Set the minimum number of falling edges required at the SYN input
    /// until the conversion is terminated. Only operational in SYND mode.
    /// @param numEdges Number of edges prior to terminating conversion in SYND
    /// mode. 0 is not allowed, 1 is the minimum.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setNumEdges(const uint8_t &numEdges);

    /// @brief Getter for the current power state.
    /// @return Sensor's power state.
    bool getPowerDownState(void);

    /// @brief Sets the power state of the sensor.
    /// @param pd Power state to set.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setPowerDownState(const bool &pd);

    /// @brief Getter for the current operational state.
    /// @return Sensor's operational mode.
    as7331_dev_op_state_t getOperationMode(void);

    /// @brief Set the sensor's operating mode.
    /// @param opMode Operating mode to set.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setOperationMode(const as7331_dev_op_state_t &opMode);

    /// @brief Getter for the current measurement state.
    /// @return Sensor's measurement state.
    as7331_meas_mode_t getMeasurementMode(void);

    /// @brief Sets the sensor's measurement mode.
    /// @param measMode Measurement mode to set.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setMeasurementMode(const as7331_meas_mode_t &measMode);

    /// @brief Getter for the current standby state.
    /// @return Sensor's standby state.
    bool getStandbyState(void);

    /// @brief Sets the sensor's standby mode.
    /// @param standby State to set.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setStandbyState(const bool &standby);

    /// @brief Getter for the current start state.
    /// @return Sensor's start state.
    bool getStartState(void);

    /// @brief Sets the sensor's start state. This begins measurement.
    /// @param startState Start state to set.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setStartState(const bool &startState);

    /// @brief Gets the sensor's status when in measurement operation mode.
    /// @param statusReg Pointer to a register struct to store the sensor's
    /// current status.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getStatus(sfe_as7331_reg_meas_osr_status_t &statusReg);

    /// @brief Gets the operational state register when in configuration operation
    /// mode.
    /// @param osrReg Pointer to a register struct to store the sensor's current
    /// OSR register.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getOSR(sfe_as7331_reg_cfg_osr_t &osrReg);

    /// @brief Sets the operational state register when in configuration operation
    /// mode.
    /// @param osrReg Pointer to a register struct that has the new register
    /// configuration.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setOSR(const sfe_as7331_reg_cfg_osr_t &osrReg);

    /// @brief Gets the configuration register #1 when in configuration operation
    /// mode.
    /// @param creg1 Pointer to a register struct to store the sensor's current
    /// creg1 register.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getCReg1(sfe_as7331_reg_cfg_creg1_t &creg1);

    /// @brief Sets the configuration register #1 when in configuration operation
    /// mode.
    /// @param creg1 Pointer to a register struct that has the new register
    /// configuration.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setCReg1(const sfe_as7331_reg_cfg_creg1_t &creg1);

    /// @brief Gets the configuration register #2 when in configuration operation
    /// mode.
    /// @param creg2 Pointer to a register struct to store the sensor's current
    /// creg2 register.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getCReg2(sfe_as7331_reg_cfg_creg2_t &creg2);

    /// @brief Sets the configuration register #2 when in configuration operation
    /// mode.
    /// @param creg2 Pointer to a register struct that has the new register
    /// configuration.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setCReg2(const sfe_as7331_reg_cfg_creg2_t &creg2);

    /// @brief Gets the configuration register #3 when in configuration operation
    /// mode.
    /// @param creg3 Pointer to a register struct to store the sensor's current
    /// creg3 register.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getCReg3(sfe_as7331_reg_cfg_creg3_t &creg3);

    /// @brief Sets the configuration register #3 when in configuration operation
    /// mode.
    /// @param creg3 Pointer to a register struct that has the new register
    /// configuration.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setCReg3(const sfe_as7331_reg_cfg_creg3_t &creg3);

    /// @brief Gets the break register when in configuration operation mode.
    /// @param breakReg Pointer to a register struct to store the sensor's current
    /// break register.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getBreak(uint8_t &breakReg);

    /// @brief Sets the break register when in configuration operation mode.
    /// @param breakReg Pointer to a register struct that has the new register
    /// configuration.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setBreak(const uint8_t &breakReg);

    /// @brief Gets the edges register when in configuration operation mode.
    /// @param edgesReg Pointer to a register struct to store the sensor's current
    /// edges register.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getEdges(uint8_t &edgesReg);

    /// @brief Sets the edges register when in configuration operation mode.
    /// @param edgesReg Pointer to a register struct that has the new register
    /// configuration.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setEdges(const uint8_t &edgesReg);

    /// @brief Gets the option register when in configuration operation mode.
    /// @param optReg Pointer to a register struct to store the sensor's current
    /// option register.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t getOptIndex(sfe_as7331_reg_cfg_optreg_t &optReg);

    /// @brief Sets the option register when in configuration operation mode.
    /// @param optReg Pointer to a register struct that has the new register
    /// configuration.
    /// @return 0 if successful, negative if error, positive for warning.
    sfeTkError_t setOptIndex(const sfe_as7331_reg_cfg_optreg_t &optReg);

  private:
    /// @brief Reads a UV result register and saves it to the respective internal
    /// variable.
    /// @param uv_type The type of UV you want to read.
    /// @return 0 if successful, negative if error, positive if warning
    sfeTkError_t readRawUV(const as7331_uv_type &uv_type);

    /// @brief Converts the raw temperature value to a human readable form.
    /// @param inputVal Raw temperature value to convert.
    /// @return The converted device temperature in degree C.
    float convertRawTempToTempC(const uint16_t &inputVal);

    /// @brief Converts a raw result value and performs a conversion to a human
    /// readable format.
    /// @param rawVal The raw input value.
    /// @param convFactor Conversion factor to multiply the raw input value.
    /// @return The calculated conversion result
    float convertRawUVVal(const uint16_t &rawVal, const float &convFactor);

    /// @brief Called when changing values that affect the conversion, calculates
    /// a new conversion factor to reduce the conversion overhead.
    void calculateConversionFactors(void);

    /// @brief Called to reset all local values back to initial conditions.
    void setDefaultSettings(void);

    sfeTkIBus *_theBus;  // Pointer to bus device.

    uint8_t _breakTime; // Local config value. Value is in us/8. EX: _breakTime = 20 means 20*8 = 160us.
    uint8_t _numEdges;  // Local config value. Edges seen on SYN pin before ending conversion in SYND mode.

    bool _readyPinMode;         // Local config value. False is Push/Pull True is Open/Drain.
    bool _dividerEnabled;       // Local config value. False is disabled, True is enabled.
    bool _tempConvEnabled;      // Local config value. False is disabled, True is enabled.
    bool _indexMode;            // Local config value. False is for controllers without repeat start.
    bool _standbyState;         // Local state value. False means the device is not in standby mode.
    bool _startState;           // Local state value. False means the device is not allowed to measure.
    bool _powerDownEnableState; // Local state value. False means the device is NOT POWERED DOWN. This is inverse.

    as7331_dev_op_state_t _opMode; // Local state value. Configuration or Measure operating mode.
    as7331_gain_t _sensorGain;     // Local config value. Sensor gain stored as (1 << (11 - gain)).
    as7331_conv_clk_freq_t _cclk;  // Local config value. Sensor's conversion clock stored as 1024*(1 << cclk) kHz.
    as7331_meas_mode_t _mmode; // Local state value. Details the device's measurement mode as CONT, CMD, SYNS, or SYND.
    as7331_conv_time_t _conversionTime; // Local config value. Contains the conversion time stored as (1 << time) ms.
    as7331_divider_val_t _dividerRange; // Local config value. Contains the predivider value stored as (1 << 1 + range).

    float _uva;         // Last valid UVA result in uW/cm2.
    float _uvb;         // Last valid UVB result in uW/cm2.
    float _uvc;         // Last valid UVC result in uW/cm2.
    float _temperature; // Last valid temperature result in degC.

    uint32_t _outputConversionTime; // Last valid output conversion time result in of clock cycle count.

    float _conversionA; // Internal conversion factor for the UVA sensor.
    float _conversionB; // Internal conversion factor for the UVB sensor.
    float _conversionC; // Internal conversion factor for the UVC sensor.

    const float _fsrA = 348160.0; // Full Scale Resolution for the UVA sensor.
    const float _fsrB = 387072.0; // Full Scale Resolution for the UVB sensor.
    const float _fsrC = 169984.0; // Full Scale Resolution for the UVC sensor.
};

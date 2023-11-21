/*
    SparkFun Spectral UV Sensor - AS7331

    Qwiic 1x1
    https://www.sparkfun.com/products/
    Qwiic Mini
    https://www.sparkfun.com/products/

    Repository
    https://github.com/sparkfun/SparkFun_AS7331_Arduino_Library

    SPDX-License-Identifier: MIT

    Copyright (c) 2023 SparkFun Electronics

    Name: SparkFun_AS7331.h
    
    Description:
    The SfeAS7331ArdI2C is a templated class that defines Arduino specific 
    behavior for initializing devices. It inherits from the base C++ class 
    SfeAS7331Base which defines the functions for interacting with the AS7331 
    Spectral UV Sensor.

*/

#pragma once

#include "sfe_as7331_registers.h"

#include <Arduino.h>
#include "sfe_i2c_arduino.h"

/// @brief Measures structure to hold return values.
typedef struct {
    float uva;
    float uvb;
    float uvc;
    float temperature;
    uint32_t outputConversionTime;
} sfeAS7331_measures_t;

/// @brief Configuration struct to hold local sensor configuration.
typedef struct {
    as7331_gain_t sensorGain;
    as7331_conv_clk_freq_t cclk;
    as7331_conv_time_t conversionTime;
    as7331_ready_pin_mode_t readyPinMode;
    as7331_divider_enable_t dividerEnabled;
    as7331_divider_val_t dividerRange;
    as7331_ext_syn_temp_meas_t enableTempConv;
    as7331_simple_reg_read_mode_t indexMode;
    uint8_t breakTime;
    uint8_t numEdges;
} sfeAS7331_config_t;

/// @brief State struct to hold local sensor state.
typedef struct {
    as7331_power_state_t pd;
    as7331_device_op_state_t opMode;
    as7331_meas_mode_t mmode;
    as7331_standby_mode_t sb;
    as7331_startstate_t ss;
} sfeAS7331_state_t;

/// @brief Default measures struct for initialization and reset.
const sfeAS7331_measures_t measuresDefault = {
                .uva = (uint16_t)(-1),
                .uvb = (uint16_t)(-1),
                .uvc = (uint16_t)(-1),
                .temperature = (uint16_t)(-1),
                .outputConversionTime = (uint32_t)(-1)
            };

/// @brief Default configuration struct for initialization and reset.
const sfeAS7331_config_t configDefault = {
                .sensorGain         = GAIN_2,
                .cclk               = CCLK_1_024_MHZ,
                .conversionTime     = TIME_64MS,
                .readyPinMode       = READYPIN_PUSHPULL,
                .dividerEnabled     = DIVIDER_DISABLED,
                .dividerRange       = DIV_2,
                .enableTempConv     = SYN_TEMP_ENABLED,
                .indexMode          = INDEX_REPEAT_START,
                .breakTime          = 25, // 25 * 8us = 200us
                .numEdges           = 1
            };

/// @brief Default state struct for initialization and reset.
const sfeAS7331_state_t stateDefault = { 
                .pd      = POWER_DOWN_ENABLE,
                .opMode  = DEVICE_MODE_CFG,
                .mmode   = MEAS_MODE_CMD,
                .sb      = STANDBY_DISABLED,
                .ss      = START_STATE_DISABLED
            };

/// @class SfeAS7331Base
/// @brief Template device class for the AS7331 Spectral UV Sensor.
template<typename sfeBusDevice, typename sfeBusDeviceSettings>
class SfeAS7331Base {
    public:
        SfeAS7331Base(){
            _state = stateDefault;
            
            _config = configDefault;

            measures = measuresDefault;
        };

        /// @brief This method is called to initialize the AS7331 device through the specified bus.
        /// @param theBus Pointer to the bus object.
        /// @param devSettings Pointer to the device settings object.
        /// @param deviceAddress I2C address for the device.
        /// @return True if successful, false if it fails.
        bool begin(sfeBusDevice *theBus, sfeBusDeviceSettings *devSettings, const uint8_t deviceAddress = AS7331_ADDR_DEFAULT)
        {
            setCommunicationBus(theBus, devSettings);
            return begin(deviceAddress);
        }

        /// @brief This method is called to initialize the AS7331 device through the specified bus.
        /// @param theBus Pointer to the bus object.
        /// @param deviceAddress I2C address for the device.
        /// @return True if successful, false if it fails.
        bool begin(sfeBusDevice *theBus, uint8_t deviceAddress = AS7331_ADDR_DEFAULT)
        {
            setCommunicationBus(theBus);
            return begin(deviceAddress);
        }

        /// @brief This method is called to initialize the AS7331 device at the specified bus address.
        /// @param deviceAddress I2C address for the device.
        /// @return True if successful, false if it fails.
        bool begin(uint8_t deviceAddress = AS7331_ADDR_DEFAULT)
        {
            if(!_sfeBus)
                setCommunicationBus();

            setDeviceAddress(deviceAddress);

            _sfeBus->begin();

            return init();
        }

        /// @brief Finds the device and initializes the default state.
        /// @param runSetup Flag to run the default setup if set.
        /// @return True if successful, false if it fails.
        bool init(bool runSetup = true)
        {
            if(SFE_BUS_OK != _sfeBus->ping(_devSettings))
                return false;
            
            reset();

            if(!isConnected())
                return false;

            if(runSetup)
                return runDefaultSetup();
            
            return true;
        }

        /// @brief Writes to the data to the given register.
        /// @param  offset The register to write to.
        /// @param  data The data to write to the register.
        /// @param  length The number of writes
        /// @return The SFE_BUS status code on the write to the given register.
        int8_t writeRegister(const uint8_t offset, const uint8_t *data, const uint16_t length = 1)
        {
            return _sfeBus->writeRegisterBytes((SFEBusDevSettings*)_devSettings, offset, data, length);
        }

        /// @brief Reads data from the specified register.
        /// @param  offset The register to read from.
        /// @param  data The pointer to the value to store the value.
        /// @param  length The number of reads
        /// @return The SFE_BUS status code on the read to the given register.
        int8_t readRegister(const uint8_t offset, uint8_t *data, const uint16_t length = 1)
        {
            return _sfeBus->readRegisterBytes((SFEBusDevSettings*)_devSettings, offset, data, length);
        }

        /// @brief Checks to see if the AS7331 is connected.
        /// @return True if successful, false otherwise.
        bool isConnected(void)
        {
            return (AS7331_DEFAULT_DEV_ID == getDeviceID());
        }

        /// @brief Requests the device ID from the sensor.
        /// @return The device ID of the sensor.`
        uint8_t getDeviceID(void)
        {
            uint8_t devID;

            if(SFE_BUS_OK != readRegister(SFE_AS7331_REGISTER_CFG_AGEN, &devID))
                return 0;
            
            return devID;
        }

        /// @brief Helper class that sets up the sensor and state in the POR configuration.
        /// @param runSoftReset Flag that runs the soft reset function to reset the device.
        /// @return True if successful, false otherwise.
        bool runDefaultSetup(bool runSoftReset = false)
        {
            if(runSoftReset)
                return reset();
            else
            {
                _state = stateDefault;
                _config = configDefault;
                measures = measuresDefault;
            }

            sfe_as7331_reg_cfg_osr_t osr;
            if(SFE_BUS_OK != getOSR(&osr))
                return false;
            
            uint8_t regs[6];
            if(SFE_BUS_OK != readRegister(SFE_AS7331_REGISTER_CFG_CREG1, regs, 6U))
                return false;

            sfe_as7331_reg_cfg_creg1_t creg1 = {.byte = regs[0]};
            sfe_as7331_reg_cfg_creg2_t creg2 = {.byte = regs[1]};
            sfe_as7331_reg_cfg_creg3_t creg3 = {.byte = regs[2]};
            sfe_as7331_reg_cfg_break_t breakreg = regs[3];
            sfe_as7331_reg_cfg_edges_t edgesreg = regs[4];
            sfe_as7331_reg_cfg_optreg_t optreg = {.byte = regs[5]};

            osr.ss = _state.ss;
            osr.pd = _state.pd;
            osr.dos = _state.opMode;

            creg1.gain = _config.sensorGain;
            creg1.time = _config.conversionTime;

            creg2.en_tm = _config.enableTempConv;
            creg2.en_div = _config.dividerEnabled;
            creg2.div = _config.dividerRange;

            creg3.mmode = _state.mmode;
            creg3.sb = _state.sb;
            creg3.rdyod = _config.readyPinMode;
            creg3.cclk = _config.cclk;

            breakreg = _config.breakTime;

            edgesreg = _config.numEdges;

            optreg.init_idx = _config.indexMode;

            if(SFE_BUS_OK != setOSR(&osr))
                return false;
            
            regs[0] = creg1.byte;
            regs[1] = creg2.byte;
            regs[2] = creg3.byte;
            regs[3] = breakreg;
            regs[4] = edgesreg;
            regs[5] = optreg.byte;

            if(SFE_BUS_OK != writeRegister(SFE_AS7331_REGISTER_CFG_CREG1, regs, 6U))
                return false;

            calculateConversionFactors();
            
            return true;
        }

        /// @brief Puts the sensor in the specified measurement mode.
        /// @param measMode Measurement mode to enter.
        /// @param startMeasure Flag to start measuring immediately if set.
        /// @return True if successful, false otherwise.
        bool startMeasurement(const as7331_meas_mode_t measMode = MEAS_MODE_CONT, bool startMeasure = false)
        {
            if(_state.pd == POWER_DOWN_ENABLE)
                if(SFE_BUS_OK != setPowerState(POWER_DOWN_DISABLE)) return false;

            if(_state.opMode != DEVICE_MODE_MEAS)
                if(SFE_BUS_OK != setOperationMode(DEVICE_MODE_MEAS)) return false;

            if(_state.mmode != measMode)
            {
                if(SFE_BUS_OK != setStandbyMode(STANDBY_DISABLED)) 
                    return false;
                if(SFE_BUS_OK != setMeasurementMode(measMode)) 
                    return false;
            }

            if(startMeasure)
                if(SFE_BUS_OK != setStartStateMode(START_STATE_ENABLED)) return false;

            return true;
        }

        /// @brief Sets the communication bus to the specified bus and device settings.
        /// @param theBus Bus to set as the communication device.
        /// @param deviceSettings Device settings to use when setting the bus for this sensor.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setCommunicationBus(sfeBusDevice *theBus, sfeBusDeviceSettings *deviceSettings)
        {
            int8_t result = setCommunicationBus(theBus);
            if(SFE_BUS_OK != result)
                return result;

            return setCommunicationDevSettings(deviceSettings);
        }

        /// @brief Sets teh communication bus to the specified bus. Creates new instance if none specified.
        /// @param theBus Bus to set as the communication devie.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setCommunicationBus(sfeBusDevice *theBus = nullptr)
        {
            if(theBus == nullptr)
                theBus = new sfeBusDevice();

            if(!theBus)
                return SFE_BUS_E_NULL_PTR;

            _sfeBus = theBus;

            return SFE_BUS_OK;
        }

        /// @brief Sets settings of this sensor's device.
        /// @param deviceSettings Settings structure for this device. Creates new instance if none specified.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setCommunicationDevSettings(sfeBusDeviceSettings *deviceSettings = nullptr)
        {
            if(deviceSettings == nullptr)
                deviceSettings = new sfeBusDeviceSettings();

            if(!deviceSettings)
                return SFE_BUS_E_NULL_DEV_SETTINGS;

            _devSettings = deviceSettings;

            return SFE_BUS_OK;
        }

        /// @brief Sets the address that the bus uses to communicate with the sensor.
        /// @param deviceAddress Device address to use.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setDeviceAddress(const uint8_t deviceAddress)
        {
            int8_t result = SFE_BUS_OK;

            if(!_devSettings)
                result = setCommunicationDevSettings();

            if(SFE_BUS_OK != result)
                return result;

            _devSettings->devAddr = deviceAddress;

            return SFE_BUS_OK;
        }

        /// @brief Performs a soft reset of the sensor.
        /// @return True if successful, false otherwise.
        bool reset(void)
        {
            sfe_as7331_reg_cfg_osr_t osr;
            
            
            if(SFE_BUS_OK != getOSR(&osr))
                return false;

            osr.sw_res = 1;

            if(SFE_BUS_OK != setOSR(&osr))
                return false;

            _state = stateDefault;
            _config = configDefault;
            measures = measuresDefault;

            return true;
        }

        /// @brief Reads the sensor's temperature, converts it to a usable form, and saves it to the measures struct.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t readTemp(void)
        {
            uint8_t tempRaw[2];
            
            int8_t result = readRegister(SFE_AS7331_REGISTER_MEAS_TEMP, tempRaw, 2U);

            if(SFE_BUS_OK != result)
                return result;

            measures.temperature = convertRawTempToTempC((uint16_t)(((uint16_t)tempRaw[1] << 8 | tempRaw[0])));

            return SFE_BUS_OK;
        }

        /// @brief Reads the sensor's UVA register, converts it to a usable form, and saves it to the measures struct.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t readUVA(void)
        {
            uint8_t uvaRaw[2];

            int8_t result = readRegister(SFE_AS7331_REGISTER_MEAS_MRES1, uvaRaw, 2U);

            if(SFE_BUS_OK != result)
                return result;

            measures.uva = ((float)((uint16_t)((uint16_t)uvaRaw[1] << 8 | uvaRaw[0]))-1.0f)*_conversionA;
            
            return SFE_BUS_OK;
        }

        /// @brief Reads the sensor's UVB register, converts it to a usable form, and saves it to the measures struct.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t readUVB(void)
        {
            uint8_t uvbRaw[2];

            int8_t result = readRegister(SFE_AS7331_REGISTER_MEAS_MRES2, uvbRaw, 2U);

            if(SFE_BUS_OK != result)
                return result;

            measures.uvb = ((float)((uint16_t)((uint16_t)uvbRaw[1] << 8 | uvbRaw[0]))-1.0f)*_conversionB;
            
            return SFE_BUS_OK;
        }

        /// @brief Reads the sensor's UVC register, converts it to a usable form, and saves it to the measures struct.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t readUVC(void)
        {
            uint8_t uvcRaw[2];

            int8_t result = readRegister(SFE_AS7331_REGISTER_MEAS_MRES3, uvcRaw, 2U);

            if(SFE_BUS_OK != result)
                return result;

            measures.uvc = ((float)((uint16_t)((uint16_t)uvcRaw[1] << 8 | uvcRaw[0]))-1.0f)*_conversionC;

            return SFE_BUS_OK;
        }

        /// @brief Read's all three UV registers, converts them to a usable form, then saves them to the measures struct.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t readAllUV(void)
        {
            uint8_t dataRaw[6];

            int8_t result = readRegister(SFE_AS7331_REGISTER_MEAS_MRES1, dataRaw, 6U);

            if(SFE_BUS_OK != result)
                return result;

            measures.uva = (float)((uint16_t)(((uint16_t)dataRaw[1]) << 8 | dataRaw[0])-1.0f)*_conversionA;
            measures.uvb = (float)((uint16_t)(((uint16_t)dataRaw[3]) << 8 | dataRaw[2])-1.0f)*_conversionB;
            measures.uvc = (float)((uint16_t)(((uint16_t)dataRaw[5]) << 8 | dataRaw[4])-1.0f)*_conversionC;

            return SFE_BUS_OK;
        }

        /// @brief Read the sensor's temperature, UV, and external time conversion clock counts, converts them, and then saves them to the measures struct.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t readAll(void)
        {
            uint8_t dataRaw[12];

            int8_t result = readRegister(SFE_AS7331_REGISTER_MEAS_TEMP, dataRaw, 12U);

            if(SFE_BUS_OK != result)
                return result;

            measures.uva = (float)((uint16_t)(((uint16_t)dataRaw[1]) << 8 | dataRaw[0]))*_conversionA;
            measures.uvb = (float)((uint16_t)(((uint16_t)dataRaw[3]) << 8 | dataRaw[2]))*_conversionB;
            measures.uvc = (float)((uint16_t)(((uint16_t)dataRaw[5]) << 8 | dataRaw[4]))*_conversionC;
            measures.temperature = convertRawTempToTempC((uint16_t)(((uint16_t)dataRaw[7] << 8 | dataRaw[6])));
            measures.outputConversionTime = (uint32_t)(((uint32_t)dataRaw[11] << 24) | ((uint32_t)dataRaw[10] << 16) | ((uint32_t)dataRaw[9] << 8) | dataRaw[8]);

            return SFE_BUS_OK;
        }

        /// @brief Read the conversion clock counts register and saves it to the measures struct.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t readOutConv(void)
        {
            uint8_t tconvRaw[4];

            int8_t result = readRegister(SFE_AS7331_REGISTER_MEAS_OUTCONV_L, tconvRaw, 4U);

            if(SFE_BUS_OK != result)
                return result;

            measures.outputConversionTime = (uint32_t)(((uint32_t)tconvRaw[3] << 24) | ((uint32_t)tconvRaw[2] << 16) | ((uint32_t)tconvRaw[1] << 8) | tconvRaw[0]);

            return SFE_BUS_OK;
        }

        /// @brief Getter function to get the current internal state.
        /// @return internal state struct.
        sfeAS7331_state_t getState(void)
        {
            return _state;
        }

        /// @brief Getter function to get the current internal configuration.
        /// @return internal configuration struct.
        sfeAS7331_config_t getConfig(void)
        {
            return _config;
        }

        /// @brief Sets the UV sensor's gain.
        /// @param gain The gain to set the sensor to.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setGain(const as7331_gain_t gain)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg1_t creg1;

            result = getCReg1(&creg1);
            if(SFE_BUS_OK != result)
                return result;

            creg1.gain = gain;

            result = setCReg1(&creg1);
            if(SFE_BUS_OK != result)
                return result;

            _config.sensorGain = gain;

            calculateConversionFactors();

            return SFE_BUS_OK;
        }

        /// @brief Sets the conversion time that the sensor will run to.
        /// @param convTime Conversion time to set the sensor to.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setConversionTime(const as7331_conv_time_t convTime)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg1_t creg1;

            result = getCReg1(&creg1);
            if(SFE_BUS_OK != result)
                return result;

            creg1.time = convTime;

            result = setCReg1(&creg1);
            if(SFE_BUS_OK != result)
                return result;

            _config.conversionTime = convTime;

            calculateConversionFactors();

            return SFE_BUS_OK;
        }

        /// @brief Set the sensor's internal clock speed.
        /// @param cclk Clock speed to set on the sensor.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setCClk(const as7331_conv_clk_freq_t cclk)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg3_t creg3;

            result = getCReg1(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            creg3.cclk = cclk;

            result = setCReg1(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            _config.cclk = cclk;

            calculateConversionFactors();

            return SFE_BUS_OK;
        }

        /// @brief Enables or disables the internal UV result divider.
        /// @param isEnabled Enable or disable the divder.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setEnableDivider(const as7331_divider_enable_t isEnabled)
        {
            if(_config.dividerEnabled == isEnabled)
                return SFE_BUS_OK;
            
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg2_t creg2;

            result = getCReg2(&creg2);
            if(SFE_BUS_OK != result)
                return result;

            creg2.en_div = isEnabled;

            result = setCReg2(&creg2);
            if(SFE_BUS_OK != result)
                return result;

            _config.dividerEnabled = isEnabled;

            if(isEnabled == DIVIDER_ENABLED)
                calculateConversionFactors();

            return SFE_BUS_OK;
        }

        /// @brief Sets the value of the internal UV result divider.
        /// @param divider Divider value to set.
        /// @param setEnableDiv Option to turn on the divider if desired.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setDigitalDivider(const as7331_divider_val_t divider, const bool setEnableDiv = false)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg2_t creg2;

            result = getCReg2(&creg2);
            if(SFE_BUS_OK != result)
                return result;

            creg2.div = divider;

            result = setCReg2(&creg2);
            if(SFE_BUS_OK != result)
                return result;

            _config.dividerRange = divider;

            if(setEnableDiv)
                result = setEnableDivider(DIVIDER_ENABLED);
            else
                calculateConversionFactors();

            return SFE_BUS_OK;
        }

        /// @brief Sets the ready pin type to push-pull or open-drain.
        /// @param pinMode Mode to set the ready pin to.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setReadyPinMode(const as7331_ready_pin_mode_t pinMode)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg3_t creg3;

            result = getCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            creg3.rdyod = pinMode;

            result = setCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            _config.readyPinMode = pinMode;

            return SFE_BUS_OK;
        }

        /// @brief Enables or disables the temperature conversion when in the SYND mode.
        /// @param isEnabled Enable or disable the feature.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setEnableTemperatureConversion(const as7331_ext_syn_temp_meas_t isEnabled)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg2_t creg2;

            result = getCReg2(&creg2);
            if(SFE_BUS_OK != result)
                return result;

            creg2.en_tm = isEnabled;

            result = setCReg2(&creg2);
            if(SFE_BUS_OK != result)
                return result;

            _config.enableTempConv = isEnabled;

            return SFE_BUS_OK;
        }

        /// @brief Set the index mode for compatibility with I2C controllers that don't support repeated start.
        /// @param indexMode Simple or standard I2C addressing mode.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setIndexMode(const as7331_simple_reg_read_mode_t indexMode)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_optreg_t optreg;

            result = getOptIndex(&optreg);
            if(SFE_BUS_OK != result)
                return result;

            optreg.init_idx = indexMode;

            result = setOptIndex(&optreg);
            if(SFE_BUS_OK != result)
                return result;

            _config.indexMode = indexMode;

            return SFE_BUS_OK;
        }

        /// @brief Set the minimum break time between measurements in CONT, SYNS, and SYND modes.
        /// @param breakTime Time between measurements, 8us step time, max 2048us. A 0 value is a minimum of 3 fclk cycles.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setBreakTime(const uint8_t breakTime)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_break_t breakreg;

            result = getBreak(&breakreg);
            if(SFE_BUS_OK != result)
                return result;

            breakreg = breakTime;

            result = setBreak(&breakreg);
            if(SFE_BUS_OK != result)
                return result;

            _config.breakTime = breakTime;

            return SFE_BUS_OK;
        }

        /// @brief Set the minimum number of falling edges required at the SYN input until the conversion is terminated. Only operational in SYND mode.
        /// @param numEdges Number of edges prior to terminating conversion in SYND mode. 0 is not allowed, 1 is the minimum.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setNumEdges(const uint8_t numEdges)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_edges_t edgesreg;

            result = getEdges(&edgesreg);
            if(SFE_BUS_OK != result)
                return result;

            edgesreg = numEdges;

            result = setEdges(&edgesreg);
            if(SFE_BUS_OK != result)
                return result;

            _config.numEdges = numEdges;

            return SFE_BUS_OK;
        }

        /// @brief Getter for the currently configured gain.
        /// @return Sensor's gain expressed as (1 << (11 - gain)).
        as7331_gain_t getConfigGain(void)
        {
            return _config.sensorGain;
        }

        /// @brief Getter for the currently configured conversion clock.
        /// @return Sensor's conversion clock expressed as 1024*(1 << cclk).
        as7331_conv_clk_freq_t getConfigCClk(void)
        {
            return _config.cclk;
        }

        /// @brief Getter for the currently configured conversion time.
        /// @return Sensor's conversion time expressed as (1 << time).
        as7331_conv_time_t getConfigConversionTime(void)
        {
            return _config.conversionTime;
        }

        /// @brief Getter for the currently configured pin mode.
        /// @return Sensor's ready pin configuration.
        as7331_ready_pin_mode_t getConfigReadyPinMode(void)
        {
            return _config.readyPinMode;
        }

        /// @brief Getter for the currently configured divider status.
        /// @return Whether the sensor's divider is enabled or disabled.
        as7331_divider_enable_t getConfigDividerEnabled(void)
        {
            return _config.dividerEnabled;
        }

        /// @brief Getter for the currently configured divider range.
        /// @return Sensor's internal UV predivider range.
        as7331_divider_val_t getConfigDividerRange(void)
        {
            return _config.dividerRange;
        }

        /// @brief Getter for the currently configured SYND temperature conversion status.
        /// @return Whether the sensor will output temperature in SYND mode.
        as7331_ext_syn_temp_meas_t getConfigExternalSyncTempConversion(void)
        {
            return _config.enableTempConv;
        }

        /// @brief Getter for the currently configured I2C compatibility mode.
        /// @return Sensor's configuration for interacting with simpler I2C controllers.
        as7331_simple_reg_read_mode_t getConfigIndexMode(void)
        {
            return _config.indexMode;
        }

        /// @brief Getter for the currently configured minimum break time in CONT, CMD, SYNS modes.
        /// @return Sensor's breaktime in 8us steps.
        uint8_t getConfigBreakTime(void)
        {
            return _config.breakTime;
        }

        /// @brief Getter for the currently configured minimum number of edges to end conversion when in SYND mode.
        /// @return Sensor's minimum number of edges, minimum 1 edge.
        uint8_t getConfigNumEdges(void)
        {
            return _config.numEdges;
        }

        /// @brief Gets the current power state from the sensor.
        /// @param pd Pointer to an enum to store the current power state.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getPowerState(as7331_power_state_t *pd)
        {
            int8_t result = SFE_BUS_OK;
            
            sfe_as7331_reg_cfg_osr_t osr;

            result = getOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;

            _state.pd = (as7331_power_state_t)osr.pd;
            *pd = _state.pd;
            
            return SFE_BUS_OK;
        }

        /// @brief Sets the power state of the sensor.
        /// @param pd Power state to set.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setPowerState(const as7331_power_state_t pd = POWER_DOWN_DISABLE)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_osr_t osr;
            
            result = getOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;

            osr.pd = pd;
            
            result = setOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;

            _state.pd = pd;

            return SFE_BUS_OK;
        }

        /// @brief Gets the current operation mode from the sensor.
        /// @param opMode Pointer to an enum to store the current operating mode.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getOperationMode(as7331_device_op_state_t *opMode)
        {
            sfe_as7331_reg_cfg_osr_t osr;
            int8_t result = getOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;

            _state.opMode = (as7331_device_op_state_t)osr.dos;
            *opMode = _state.opMode;
            
            return SFE_BUS_OK;
        }

        /// @brief Set the sensor's operating mode.
        /// @param opMode Operating mode to set.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setOperationMode(const as7331_device_op_state_t opMode)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_osr_t osr;

            result = getOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;
            
            osr.dos = opMode;

            result = setOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;

            _state.opMode = opMode;

            return SFE_BUS_OK;
        }

        /// @brief Gets the sensor's current measurement mode.
        /// @param measMode Pointer to an enum to store the current measurment mode.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getMeasurementMode(as7331_meas_mode_t *measMode)
        {
            sfe_as7331_reg_cfg_creg3_t creg3;
            int8_t result = getCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            _state.mmode = (as7331_meas_mode_t)creg3.mmode;
            *measMode = _state.mmode;
            
            return SFE_BUS_OK;
        }

        /// @brief Sets the sensor's measurement mode.
        /// @param measMode Measurement mode to set.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setMeasurementMode(const as7331_meas_mode_t measMode)
        {
            int8_t result = SFE_BUS_OK;
            
            sfe_as7331_reg_cfg_creg3_t creg3;
            
            result = getCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            creg3.mmode = measMode;

            result = setCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            _state.mmode = measMode;
            
            return SFE_BUS_OK;
        }

        /// @brief Gets the sensor's current standby mode state.
        /// @param standby Pointer to an enum to store the current standby state.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getStandbyMode(as7331_standby_mode_t *standby)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_creg3_t creg3;

            result = getCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            _state.sb = (as7331_standby_mode_t)creg3.sb;
            *standby = _state.sb;

            return SFE_BUS_OK;
        }

        /// @brief Sets the sensor's standby mode.
        /// @param standby State to set.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setStandbyMode(const as7331_standby_mode_t standby)
        {
            int8_t result = SFE_BUS_OK;
            
            sfe_as7331_reg_cfg_creg3_t creg3;
            
            result = getCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            creg3.sb = standby;

            result = setCReg3(&creg3);
            if(SFE_BUS_OK != result)
                return result;

            _state.sb = standby;
            
            return SFE_BUS_OK;
        }

        /// @brief Gets the sensor's current start state.
        /// @param startState Pointer to an enum to store the current start state.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getStartStateMode(as7331_startstate_t *startState)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_osr_t osr;

            result = getOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;

            _state.ss = (as7331_startstate_t)osr.ss;
            *startState = _state.ss;

            return SFE_BUS_OK;
        }

        /// @brief Sets the sensor's start state. This begins measurement.
        /// @param startState Start state to set.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setStartStateMode(const as7331_startstate_t startState)
        {
            int8_t result = SFE_BUS_OK;

            sfe_as7331_reg_cfg_osr_t osr;

            result = getOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;
            
            osr.ss = startState;

            result = setOSR(&osr);
            if(SFE_BUS_OK != result)
                return result;

            _state.ss = startState;

            return SFE_BUS_OK;
        }

        /// @brief Getter for the current power state.
        /// @return Sensor's power state.
        as7331_power_state_t getStatePower(void)
        {
            return _state.pd;
        }

        /// @brief Getter for the current operational state.
        /// @return Sensor's operational state.
        as7331_device_op_state_t getStateOperational(void)
        {
            return _state.opMode;
        }

        /// @brief Getter for the current measurement state.
        /// @return Sensor's measurement state.
        as7331_meas_mode_t getStateMeasurement(void)
        {
            return _state.mmode;
        }

        /// @brief Getter for the current standby state.
        /// @return Sensor's standby state.
        as7331_standby_mode_t getStateStandby(void)
        {
            return _state.sb;
        }

        /// @brief Getter for the current start state.
        /// @return Sensor's start state.
        as7331_startstate_t getStateStart(void)
        {
            return _state.ss;
        }

        /// @brief Gets the sensor's status when in measurement operation mode.
        /// @param statusReg Pointer to a register struct to store the sensor's current status.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getStatus(sfe_as7331_reg_meas_osr_status_t *statusReg)
        {
            return readRegister(SFE_AS7331_REGISTER_MEAS_OSR_STATUS, statusReg, 2U);
        }

        /// @brief Gets the operational state register when in configuration operation mode.
        /// @param osrReg Pointer to a register struct to store the sensor's current OSR register. 
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getOSR(sfe_as7331_reg_cfg_osr_t *osrReg)
        {
            return readRegister(SFE_AS7331_REGISTER_CFG_OSR, &osrReg->byte);
        }

        /// @brief Sets the operational state register when in configuration operation mode.
        /// @param osrReg Pointer to a register struct that has the new register configuration.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setOSR(const sfe_as7331_reg_cfg_osr_t *osrReg)
        {
            return writeRegister(SFE_AS7331_REGISTER_CFG_OSR, &osrReg->byte);
        }

        /// @brief Gets the configuration register #1 when in configuration operation mode.
        /// @param creg1 Pointer to a register struct to store the sensor's current creg1 register. 
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getCReg1(sfe_as7331_reg_cfg_creg1_t *creg1)
        {
            return readRegister(SFE_AS7331_REGISTER_CFG_CREG1, &creg1->byte);
        }

        /// @brief Sets the configuration register #1 when in configuration operation mode.
        /// @param creg1 Pointer to a register struct that has the new register configuration.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setCReg1(const sfe_as7331_reg_cfg_creg1_t *creg1)
        {
            return writeRegister(SFE_AS7331_REGISTER_CFG_CREG1, &creg1->byte);
        }

        /// @brief Gets the configuration register #2 when in configuration operation mode.
        /// @param creg2 Pointer to a register struct to store the sensor's current creg2 register. 
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getCReg2(sfe_as7331_reg_cfg_creg2_t *creg2)
        {
            return readRegister(SFE_AS7331_REGISTER_CFG_CREG2, &creg2->byte);
        }

        /// @brief Sets the configuration register #2 when in configuration operation mode.
        /// @param creg2 Pointer to a register struct that has the new register configuration.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setCReg2(const sfe_as7331_reg_cfg_creg2_t *creg2)
        {
            return writeRegister(SFE_AS7331_REGISTER_CFG_CREG2, &creg2->byte);
        }

        /// @brief Gets the configuration register #3 when in configuration operation mode.
        /// @param creg3 Pointer to a register struct to store the sensor's current creg3 register. 
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getCReg3(sfe_as7331_reg_cfg_creg3_t *creg3)
        {
            return readRegister(SFE_AS7331_REGISTER_CFG_CREG3, &creg3->byte);
        }

        /// @brief Sets the configuration register #3 when in configuration operation mode.
        /// @param creg3 Pointer to a register struct that has the new register configuration.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setCReg3(const sfe_as7331_reg_cfg_creg3_t *creg3)
        {
            return writeRegister(SFE_AS7331_REGISTER_CFG_CREG3, &creg3->byte);
        }

        /// @brief Gets the break register when in configuration operation mode.
        /// @param breakReg Pointer to a register struct to store the sensor's current break register. 
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getBreak(sfe_as7331_reg_cfg_break_t *breakReg)
        {
            return readRegister(SFE_AS7331_REGISTER_CFG_BREAK, breakReg);
        }

        /// @brief Sets the break register when in configuration operation mode.
        /// @param breakReg Pointer to a register struct that has the new register configuration.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setBreak(const sfe_as7331_reg_cfg_break_t *breakReg)
        {
            return writeRegister(SFE_AS7331_REGISTER_CFG_BREAK, breakReg);
        }

        /// @brief Gets the edges register when in configuration operation mode.
        /// @param edgesReg Pointer to a register struct to store the sensor's current edges register. 
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getEdges(sfe_as7331_reg_cfg_edges_t *edgesReg)
        {
            return readRegister(SFE_AS7331_REGISTER_CFG_EDGES, edgesReg);
        }


        /// @brief Sets the edges register when in configuration operation mode.
        /// @param edgesReg Pointer to a register struct that has the new register configuration.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setEdges(const sfe_as7331_reg_cfg_edges_t *edgesReg)
        {
            return writeRegister(SFE_AS7331_REGISTER_CFG_EDGES, edgesReg);
        }

        /// @brief Gets the option register when in configuration operation mode.
        /// @param optReg Pointer to a register struct to store the sensor's current option register. 
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t getOptIndex(sfe_as7331_reg_cfg_optreg_t *optReg)
        {
            return readRegister(SFE_AS7331_REGISTER_CFG_OPTREG, &optReg->byte);
        }

        /// @brief Sets the option register when in configuration operation mode.
        /// @param optReg Pointer to a register struct that has the new register configuration.
        /// @return 0 if successful, negative if error, positive for warning.
        int8_t setOptIndex(const sfe_as7331_reg_cfg_optreg_t *optReg)
        {
            return writeRegister(SFE_AS7331_REGISTER_CFG_OPTREG, &optReg->byte);
        }

        sfeAS7331_measures_t measures;

    private:
        /// @brief Converts the raw temperature value to a human readable form.
        /// @param inputVal Raw temperature value to convert.
        /// @return The converted device temperature in degree C.
        float convertRawTempToTempC(const uint16_t *inputVal)
        {
            // T_chip = TEMP*0.05 - 66.9
            // EX: TEMP=0x922 aka TEMP=0d2338, returns 50.0
            return ((float)(*inputVal) * 0.05f) - 66.9f;
        }

        /// @brief Called when changing values that affect the conversion, calculates a new conversion factor to reduce the conversion overhead.
        void calculateConversionFactors(void)
        {
            float divFactor = (bool)_config.dividerEnabled ? (float)(1 << (1+_config.dividerRange)) : 1.0f;
            _conversionA = fsrA/((float)(1 << (11 - _config.sensorGain)))/((float)(1 << _config.conversionTime))/((float)(1024.0 * (1 << _config.cclk)))*divFactor;
            _conversionB = fsrB/((float)(1 << (11 - _config.sensorGain)))/((float)(1 << _config.conversionTime))/((float)(1024.0 * (1 << _config.cclk)))*divFactor;
            _conversionC = fsrC/((float)(1 << (11 - _config.sensorGain)))/((float)(1 << _config.conversionTime))/((float)(1024.0 * (1 << _config.cclk)))*divFactor;
        }

        sfeBusDevice *_sfeBus; // Pointer to bus device.
        sfeBusDeviceSettings *_devSettings; // Pointer to the device settings for this sensor.

        sfeAS7331_config_t _config; // Internal sensor configuration.
        sfeAS7331_state_t _state; // Internal sensor state.

        float _conversionA; // Internal conversion factor for the UVA sensor.
        float _conversionB; // Internal conversion factor for the UVB sensor.
        float _conversionC; // Internal conversion factor for the UVC sensor.

        static constexpr float fsrA = 348160.0; // Full Scale Resolution for the UVA sensor.
        static constexpr float fsrB = 387072.0; // Full Scale Resolution for the UVB sensor.
        static constexpr float fsrC = 169984.0; // Full Scale Resolution for the UVC sensor.

};

/// @class SfeAS7331ArdI2C
/// @brief Arduino I2C implementation of the SfeAS7331Base class for the AS7331 Spectral UV Sensor.
class SfeAS7331ArdI2C : public SfeAS7331Base<SFEBusArduinoI2C, SFEBusDevSettingsI2C>
{
    /* Nothing to see here, see above. */
};

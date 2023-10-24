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
    This class defines Arduino specific class behavior for initializing 
    devices. It inherits from the base C++ class QwAS7331 which defines the 
    functions for interacting with the AS7331 Spectral UV Sensor.

*/

#pragma once

#include "sfe_as7331_registers.h"

#include <Arduino.h>
#include <SparkFun_Toolkit.h>
#include "../../SparkFun_Toolkit/src/SparkFun_Toolkit.h"

typedef struct {
    uint16_t uva;
    uint16_t uvb;
    uint16_t uvc;
    uint16_t temperature;
    uint32_t outputConversionTime;
} sfeAS7331_measures_t;

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

typedef struct {
    as7331_power_state_t pd;
    as7331_device_op_state_t opMode;
    as7331_meas_mode_t mmode;
    as7331_standby_mode_t sb;
    as7331_startstate_t ss;
} sfeAS7331_state_t;

template<typename sfeBusDevice, typename sfeBusDeviceSettings>
class SfeAS7331Base {
    public:
        SfeAS7331Base(){
            _state = { 
                .pd      = POWER_DOWN_ENABLE,
                .opMode  = DEVICE_MODE_CFG,
                .mmode   = MEAS_MODE_CMD,
                .sb      = STANDBY_DISABLED,
                .ss      = START_STATE_DISABLED
            };
            
            _config = {
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

            measures = {
                .uva = (uint16_t)(-1),
                .uvb = (uint16_t)(-1),
                .uvc = (uint16_t)(-1),
                .temperature = (uint16_t)(-1),
                .outputConversionTime = (uint32_t)(-1)
            };
        };


        bool begin(sfeBusDevice *theBus, sfeBusDeviceSettings *devSettings, uint8_t deviceAddress = AS7331_ADDR_DEFAULT)
        {
            setCommunicationBus(theBus, devSettings);
            return begin(deviceAddress);
        }


        bool begin(sfeBusDevice *theBus, uint8_t deviceAddress = AS7331_ADDR_DEFAULT)
        {
            setCommunicationBus(theBus);
            return begin(deviceAddress);
        }


        bool begin(uint8_t deviceAddress = AS7331_ADDR_DEFAULT)
        {
            if(!_sfeBus)
                setCommunicationBus();

            setDeviceAddress(deviceAddress);

            _sfeBus->begin();

            return init();
        }

        
        bool init(bool runSetup = true)
        {
            if(SFE_BUS_OK != _sfeBus->ping(_devSettings))
                return false;
            
            if(!isConnected())
                return false;
            
            if(runSetup)
                return (SFE_BUS_OK == runDefaultSetup());
            
            return true;
        }


        bool isConnected(void)
        {
            return (AS7331_DEFAULT_DEV_ID == getDeviceID());
        }


        uint8_t getDeviceID(void)
        {
            // TODO: Finish this properly.
            uint8_t devID = 0;

            return devID;
        }


        int8_t setCommunicationBus(sfeBusDevice *theBus, sfeBusDeviceSettings *deviceSettings)
        {
            int8_t result = setCommunicationBus(theBus);
            if(SFE_BUS_OK != result)
                return result;

            return setCommunicationDevSettings(deviceSettings);
        }

        
        int8_t setCommunicationBus(sfeBusDevice *theBus = nullptr)
        {
            if(theBus == nullptr)
                theBus = new sfeBusDevice();

            if(!theBus)
                return SFE_BUS_E_NULL_PTR;

            _sfeBus = theBus;

            return SFE_BUS_OK;
        }

        
        int8_t setCommunicationDevSettings(sfeBusDeviceSettings *deviceSettings = nullptr)
        {
            if(deviceSettings == nullptr)
                deviceSettings = new sfeBusDeviceSettings();

            if(!deviceSettings)
                return SFE_BUS_E_NULL_DEV_SETTINGS;

            _devSettings = deviceSettings;

            return SFE_BUS_OK;
        }

        
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

        
        int8_t writeRegister(const uint8_t offset, const uint8_t *data, const uint16_t length = 1)
        {
            return _sfeBus->writeRegisterBytes((SFEBusDevSettings*)_devSettings, offset, data, length);
        }

        
        int8_t readRegister(const uint8_t offset, uint8_t *data, const uint16_t length = 1)
        {
            return _sfeBus->readRegisterBytes((SFEBusDevSettings*)_devSettings, offset, data, length);
        }

        int8_t runDefaultSetup()
        {
            return SFE_BUS_OK;
        }


        int8_t reset(void)
        {
            // if(_state.opMode )
            return SFE_BUS_OK;
        }


        int8_t readTemp(void)
        {
            return SFE_BUS_OK;
        }


        int8_t readUVA(void)
        {
            return SFE_BUS_OK;
        }


        int8_t readUVB(void)
        {
            return SFE_BUS_OK;
        }


        int8_t readUVC(void)
        {
            return SFE_BUS_OK;
        }


        int8_t readAllUV(void)
        {
            return SFE_BUS_OK;
        }


        int8_t readAll(void)
        {
            return SFE_BUS_OK;
        }


        int8_t readOutConv(void)
        {
            return SFE_BUS_OK;
        }


        int8_t getState(sfeAS7331_state_t *state)
        {
            return SFE_BUS_OK;
        }


        int8_t getConfig(sfeAS7331_config_t *config)
        {
            return SFE_BUS_OK;
        }


        int8_t getPowerState(bool *enabled)
        {
            return SFE_BUS_OK;
        }


        int8_t setPowerState(const bool enable = true)
        {
            return SFE_BUS_OK;
        }


        int8_t getOperationMode(as7331_device_op_state_t *opMode)
        {
            return SFE_BUS_OK;
        }


        int8_t setOperationMode(const as7331_device_op_state_t opMode)
        {
            return SFE_BUS_OK;
        }


        int8_t getMeasurementMode(as7331_meas_mode_t *measMode)
        {
            return SFE_BUS_OK;
        }

        int8_t setMeasurementMode(const as7331_meas_mode_t measMode)
        {
            return SFE_BUS_OK;
        }


        int8_t getStatus(sfe_as7331_reg_meas_osr_status_t *statusReg)
        {
            return SFE_BUS_OK;
        }


        int8_t getOSR(sfe_as7331_reg_cfg_osr_t *osrReg)
        {
            return SFE_BUS_OK;
        }


        int8_t setOSR(const sfe_as7331_reg_cfg_osr_t *osrReg)
        {
            return SFE_BUS_OK;
        }


        int8_t getCReg1(sfe_as7331_reg_cfg_creg1_t *creg1)
        {
            return SFE_BUS_OK;
        }


        int8_t setCReg1(const sfe_as7331_reg_cfg_creg1_t *creg1)
        {
            return SFE_BUS_OK;
        }


        int8_t getCReg2(sfe_as7331_reg_cfg_creg2_t *creg2)
        {
            return SFE_BUS_OK;
        }


        int8_t setCReg2(const sfe_as7331_reg_cfg_creg2_t *creg2)
        {
            return SFE_BUS_OK;
        }


        int8_t getCReg3(sfe_as7331_reg_cfg_creg3_t *creg3)
        {
            return SFE_BUS_OK;
        }


        int8_t setCReg3(const sfe_as7331_reg_cfg_creg3_t *creg3)
        {
            return SFE_BUS_OK;
        }


        int8_t getBreak(sfe_as7331_reg_cfg_break_t *breakReg)
        {
            return SFE_BUS_OK;
        }


        int8_t setBreak(const sfe_as7331_reg_cfg_break_t breakReg)
        {
            return SFE_BUS_OK;
        }


        int8_t getEdges(sfe_as7331_reg_cfg_edges_t *edgesReg)
        {
            return SFE_BUS_OK;
        }


        int8_t setEdges(const sfe_as7331_reg_cfg_edges_t edgesReg)
        {
            return SFE_BUS_OK;
        }


        int8_t getOptIndex(sfe_as7331_reg_cfg_optreg_t *optReg)
        {
            return SFE_BUS_OK;
        }


        int8_t setOptIndex(const sfe_as7331_reg_cfg_optreg_t *optReg)
        {
            return SFE_BUS_OK;
        }


        sfeAS7331_measures_t measures;

    private:
        uint16_t convertRawValueToLight(const uint16_t *inputVal)
        {
            uint16_t lightVal;
            return lightVal;
        }

        sfeBusDevice *_sfeBus;
        sfeBusDeviceSettings *_devSettings;

        sfeAS7331_config_t _config;
        sfeAS7331_state_t _state;
};

class SfeAS7331ArdI2C : public SfeAS7331Base<SFEBusArduinoI2C, SFEBusDevSettingsI2C>
{
    /* Nothing to see here, see above. */
};

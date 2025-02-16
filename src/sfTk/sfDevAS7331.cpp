/**
 * @file sfDevAS7331.cpp
 * @brief Implementation file for the SparkFun AS7331 UV Sensor device driver.
 *
 * @details
 * This file implements the sfDevAS7331 class methods for configuring and reading data from
 * the AS7331 UV sensor. The driver provides a comms-agnostic interface using the SparkFun Toolkit.
 *
 * Key features:
 * - Device initialization and configuration
 * - UV sensor readings (UVA, UVB, UVC)
 * - Temperature readings
 * - Gain and clock settings
 * - Power management
 * - Measurement modes
 *
 * @section Implementation Implementation Details
 * - Register read/write operations
 * - Raw data conversion methods
 * - Device state management
 * - Configuration settings
 * - Measurement handling
 *
 * @author SparkFun Electronics
 * @date 2023
 * @copyright Copyright (c) 2023-2025, SparkFun Electronics Inc. All rights reserved.
 *
 * @section License License
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_AS7331_Arduino_Library
 */
#include "sfDevAS7331.h"

bool sfDevAS7331::begin(sfTkIBus *theBus)
{
    // Nullptr check.
    if (!_theBus && !theBus)
        return false;

    // Set the internal bus pointer, overriding current bus if it exists.
    if (theBus != nullptr)
        setCommunicationBus(theBus);

    // Get the device setup and ready.
    return runDefaultSetup();
}

uint8_t sfDevAS7331::getDeviceID(void)
{
    // Nullptr check.
    if (!_theBus)
        return 0;

    // Check sensor's operating mode.
    sfe_as7331_reg_cfg_osr_t osr;
    if (ksfTkErrOk != getOSR(osr))
        return 0;

    bool needsToBeChangedBack = false;

    // Change if necessary.
    if (osr.dos != DEVICE_MODE_CFG)
    {
        needsToBeChangedBack = true;

        osr.dos = DEVICE_MODE_CFG;

        if (ksfTkErrOk != setOSR(osr))
            return 0;
    }

    uint8_t devID;

    // Read the device ID register, if it errors then return 0.
    if (ksfTkErrOk != _theBus->readRegisterByte(kSfeAS7331RegCfgAgen, devID))
        return 0;

    // If we changed it at first, change it back.
    if (needsToBeChangedBack)
    {
        osr.dos = DEVICE_MODE_MEAS;

        if (ksfTkErrOk != setOSR(osr))
            return 0;
    }

    return devID;
}

void sfDevAS7331::setCommunicationBus(sfTkIBus *theBus)
{
    _theBus = theBus;
}

bool sfDevAS7331::runDefaultSetup(const bool &runSoftReset)
{
    // Nullptr check.
    if (!_theBus)
        return false;

    // Do we need to run a software reset?
    if (runSoftReset)
        reset();
    else
    {
        setDefaultSettings();

        // Read the OSR register by itself since the offset isn't contiguous with the rest.
        sfe_as7331_reg_cfg_osr_t osr;
        if (ksfTkErrOk != getOSR(osr))
            return false;

        // Change operation mode if necessary.
        if (osr.dos != DEVICE_MODE_CFG)
        {
            osr.dos = DEVICE_MODE_CFG;

            if (ksfTkErrOk != setOSR(osr))
                return false;
        }

        // Read all the configuration registers in.
        uint8_t regs[6];

        size_t nRead = 0;
        sfTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegCfgCreg1, regs, 6U, nRead);
        if (nRead != 6 || result != ksfTkErrOk)
            return false;

        // Assign the read in bytes to each register's byte union.
        // This allows us to address the individual bits and set them.
        sfe_as7331_reg_cfg_creg1_t creg1 = {.byte = regs[0]};
        sfe_as7331_reg_cfg_creg2_t creg2 = {.byte = regs[1]};
        sfe_as7331_reg_cfg_creg3_t creg3 = {.byte = regs[2]};
        uint8_t breakreg = regs[3];
        uint8_t edgesreg = regs[4];
        sfe_as7331_reg_cfg_optreg_t optreg = {.byte = regs[5]};

        // Here we make sure the sensor's settings match the local settings
        // by changing the sensor's settings.
        osr.ss = _startState;
        osr.pd = _powerDownEnableState;
        osr.dos = _opMode;

        creg1.gain = _sensorGain;
        creg1.time = _conversionTime;

        creg2.en_tm = _tempConvEnabled;
        creg2.en_div = _dividerEnabled;
        creg2.div = _dividerRange;

        creg3.mmode = _mmode;
        creg3.sb = _standbyState;
        creg3.rdyod = _readyPinMode;
        creg3.cclk = _cclk;

        breakreg = _breakTime;

        edgesreg = _numEdges;

        optreg.init_idx = _indexMode;

        // Write OSR first, since the offset is different from the rest.
        if (ksfTkErrOk != setOSR(osr))
            return false;

        // Assign the registers to a byte array for writing to the device.
        regs[0] = creg1.byte;
        regs[1] = creg2.byte;
        regs[2] = creg3.byte;
        regs[3] = breakreg;
        regs[4] = edgesreg;
        regs[5] = optreg.byte;

        // Write the bytes to the sensor, ensuring the device matches local settings.
        if (ksfTkErrOk != _theBus->writeRegisterRegion(kSfeAS7331RegCfgCreg1, regs, 6U))
            return false;
    }

    // Calculate new conversion factors to make sure they match the current settings.
    calculateConversionFactors();

    return true;
}

bool sfDevAS7331::prepareMeasurement(const as7331_meas_mode_t &measMode, const bool &startMeasure)
{
    // If the device is currently in power down mode.
    if (_powerDownEnableState)
    {
        if (ksfTkErrOk != setPowerDownState(false)) // Set the device to be powered up.
            return false;
    }

    // If the device is currently in standby mode.
    if (_standbyState)
    {
        if (ksfTkErrOk != setStandbyState(false)) // Set the device to not be in standby.
            return false;
    }

    // If the device's currently configured measurement mode is different than the desired mode.
    // This is just to reduce unnecessary calls.
    if (_mmode != measMode)
    {
        if (ksfTkErrOk != setMeasurementMode(measMode)) // Set the new mode.
            return false;
    }

    // If the device is in start state and we don't want it to be.
    // This prevents starting automatically in the next step.
    if (_startState && !startMeasure)
    {
        if (ksfTkErrOk != setStartState(false)) // Stop it from running automatically.
            return false;
    }

    // Now that we've configured ourselves to measure properly, move the device op mode to measure.
    if (ksfTkErrOk != setOperationMode(DEVICE_MODE_MEAS))
        return false;

    // If the device is supposed to be started automatically and isn't configured that way.
    if (!_startState && startMeasure)
    {
        if (ksfTkErrOk != setStartState(true)) // Set the device to being measuring.
            return false;
    }

    return true;
}

bool sfDevAS7331::reset(void)
{
    // OSR is available in all modes, no need to change modes.

    // Standard read-modify-write sequence.
    sfe_as7331_reg_cfg_osr_t osr;

    if (ksfTkErrOk != getOSR(osr))
        return false;

    // Set software reset bit.
    osr.sw_res = 1;

    if (ksfTkErrOk != setOSR(osr))
        return false;

    // Set internal variables back to defaults.
    setDefaultSettings();

    return true;
}

sfTkError_t sfDevAS7331::readTemp(void)
{
    // Temperature is only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return ksfTkErrFail;

    uint16_t tempRaw;

    // Read in the raw value.
    sfTkError_t result = _theBus->readRegisterWord(kSfeAS7331RegMeasTemp, tempRaw);

    if (ksfTkErrOk != result)
        return result;

    // Since temperature is more than 1 byte, need to order it correctly before conversion.
    _temperature = convertRawTempToTempC(tempRaw);

    return ksfTkErrOk;
}

sfTkError_t sfDevAS7331::readUVA(void)
{
    return readRawUV(AS7331_UVA);
}

sfTkError_t sfDevAS7331::readUVB(void)
{
    return readRawUV(AS7331_UVB);
}

sfTkError_t sfDevAS7331::readUVC(void)
{
    return readRawUV(AS7331_UVC);
}

sfTkError_t sfDevAS7331::readAllUV(void)
{
    // UV results are only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return ksfTkErrFail;

    uint8_t dataRaw[6];

    // Read in the raw data from the results registers.
    size_t nRead = 0;

    sfTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasMres1, dataRaw, 6U, nRead);

    if (nRead != 6 || result != ksfTkErrOk)
        return result;

    // If we're in SYND mode, need to calculate conversion based on the conversion time.
    if (_mmode == MEAS_MODE_SYND)
    {
        result = readOutConv();
        if (ksfTkErrOk != result)
            return result;

        // See datasheet section 7.4 Equation 4.
        //          fsrX
        //      --------------
        //      gain * outConv
        float convFactor = 1.0f / (((float)_outputConversionTime) * ((float)(1 << (11 - _sensorGain))));

        // Since result is more than 1 byte, need to order it correctly before conversion.
        // FSR is dependent on the channel being measured, so do that here.
        _uva = convertRawUVVal(((uint16_t)dataRaw[1] << 8 | dataRaw[0]) * _fsrA, convFactor);
        _uvb = convertRawUVVal(((uint16_t)dataRaw[3] << 8 | dataRaw[2]) * _fsrB, convFactor);
        _uvc = convertRawUVVal(((uint16_t)dataRaw[5] << 8 | dataRaw[4]) * _fsrC, convFactor);
    }
    else
    {
        // If we're in CONT, CMD, or SYNS mode, use the normally calculated conversion factor.
        // Since result is more than 1 byte, need to order it correctly before conversion.
        _uva = convertRawUVVal((uint16_t)dataRaw[1] << 8 | dataRaw[0], _conversionA);
        _uvb = convertRawUVVal((uint16_t)dataRaw[3] << 8 | dataRaw[2], _conversionB);
        _uvc = convertRawUVVal((uint16_t)dataRaw[5] << 8 | dataRaw[4], _conversionC);
    }

    return ksfTkErrOk;
}

sfTkError_t sfDevAS7331::readAll(void)
{
    // Results are only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return ksfTkErrFail;

    uint8_t dataRaw[8];

    size_t nRead = 0;
    sfTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasTemp, dataRaw, 8U, nRead);

    if (nRead != 8 || result != ksfTkErrOk)
        return result;

    result = readOutConv();
    if (ksfTkErrOk != result)
        return result;

    // If we're in SYND mode, need to calculate conversion based on the conversion time.
    if (_mmode == MEAS_MODE_SYND)
    {
        // See datasheet section 7.4 Equation 4.
        //          fsrX
        //      --------------
        //      gain * outConv
        float convFactor = 1.0f / (((float)_outputConversionTime) * ((float)(1 << (11 - _sensorGain))));

        // Since result is more than 1 byte, need to order it correctly before conversion.
        // FSR is dependent on the channel being measured, so do that part here.
        _uva = convertRawUVVal(((uint16_t)dataRaw[3] << 8 | dataRaw[2]) * _fsrA, convFactor);
        _uvb = convertRawUVVal(((uint16_t)dataRaw[5] << 8 | dataRaw[4]) * _fsrB, convFactor);
        _uvc = convertRawUVVal(((uint16_t)dataRaw[7] << 8 | dataRaw[6]) * _fsrC, convFactor);
    }
    else
    {
        // If we're in CONT, CMD, or SYNS mode, use the normally calculated conversion factor.
        // Since result is more than 1 byte, need to order it correctly before conversion.
        _uva = convertRawUVVal((uint16_t)dataRaw[3] << 8 | dataRaw[2], _conversionA);
        _uvb = convertRawUVVal((uint16_t)dataRaw[5] << 8 | dataRaw[4], _conversionB);
        _uvc = convertRawUVVal((uint16_t)dataRaw[7] << 8 | dataRaw[6], _conversionC);
    }

    // Since result is more than 1 byte, need to order it correctly before conversion.
    _temperature = convertRawTempToTempC((uint16_t)dataRaw[1] << 8 | dataRaw[0]);

    return ksfTkErrOk;
}

sfTkError_t sfDevAS7331::readOutConv(void)
{
    // Results are only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return ksfTkErrFail;

    uint8_t tconvRaw[4];

    size_t nRead;

    sfTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasOutConvL, tconvRaw, 4U, nRead);

    if (nRead != 4 || result != ksfTkErrOk)
        return result;

    // Since result is more than 1 byte, need to order it correctly.
    _outputConversionTime = (uint32_t)(((uint32_t)tconvRaw[3] << 24) | ((uint32_t)tconvRaw[2] << 16) |
                                       ((uint32_t)tconvRaw[1] << 8) | tconvRaw[0]);

    return ksfTkErrOk;
}

float sfDevAS7331::getUVA(void)
{
    return _uva;
}

float sfDevAS7331::getUVB(void)
{
    return _uvb;
}

float sfDevAS7331::getUVC(void)
{
    return _uvc;
}

float sfDevAS7331::getTemp(void)
{
    return _temperature;
}

uint32_t sfDevAS7331::getOutConv(void)
{
    return _outputConversionTime;
}

as7331_gain_t sfDevAS7331::getGainRaw(void)
{
    return _sensorGain;
}

uint16_t sfDevAS7331::getGainValue(void)
{
    return (1 << (11 - _sensorGain));
}

sfTkError_t sfDevAS7331::setGain(const as7331_gain_t &gain)
{
    sfe_as7331_reg_cfg_creg1_t creg1;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg1(creg1);
    if (ksfTkErrOk != result)
        return result;

    creg1.gain = gain;

    result = setCReg1(creg1);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _sensorGain = gain;

    // Gain affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return ksfTkErrOk;
}

as7331_conv_clk_freq_t sfDevAS7331::getCClkRaw(void)
{
    return _cclk;
}

uint16_t sfDevAS7331::getCClkKHz(void)
{
    return 1024 * (1 << _cclk);
}

sfTkError_t sfDevAS7331::setCClk(const as7331_conv_clk_freq_t &cclk)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    creg3.cclk = cclk;

    result = setCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _cclk = cclk;

    // CClk affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return ksfTkErrOk;
}

as7331_conv_time_t sfDevAS7331::getConversionTimeRaw(void)
{
    return _conversionTime;
}

uint16_t sfDevAS7331::getConversionTimeMillis(void)
{
    return (1 << _conversionTime);
}

sfTkError_t sfDevAS7331::setConversionTime(const as7331_conv_time_t &convTime)
{
    sfe_as7331_reg_cfg_creg1_t creg1;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg1(creg1);
    if (ksfTkErrOk != result)
        return result;

    creg1.time = convTime;

    result = setCReg1(creg1);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _conversionTime = convTime;

    // Conversion Time affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return ksfTkErrOk;
}

bool sfDevAS7331::getReadyPinMode(void)
{
    return _readyPinMode;
}

sfTkError_t sfDevAS7331::setReadyPinMode(const bool &pinMode)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    creg3.rdyod = pinMode;

    result = setCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _readyPinMode = pinMode;

    return ksfTkErrOk;
}

bool sfDevAS7331::getDigitalDividerEnabled(void)
{
    return _dividerEnabled;
}

sfTkError_t sfDevAS7331::setDigitalDividerEnabled(const bool &isEnabled)
{
    if (_dividerEnabled == isEnabled)
        return ksfTkErrOk;

    sfe_as7331_reg_cfg_creg2_t creg2;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg2(creg2);
    if (ksfTkErrOk != result)
        return result;

    creg2.en_div = isEnabled;

    result = setCReg2(creg2);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _dividerEnabled = isEnabled;

    // Digital divider affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return ksfTkErrOk;
}

as7331_divider_val_t sfDevAS7331::getDigitalDividerRange(void)
{
    return _dividerRange;
}

sfTkError_t sfDevAS7331::setDigitalDividerRange(const as7331_divider_val_t &divider, const bool &enableDiv)
{
    sfe_as7331_reg_cfg_creg2_t creg2;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg2(creg2);
    if (ksfTkErrOk != result)
        return result;

    creg2.div = divider;
    creg2.en_div = enableDiv;

    result = setCReg2(creg2);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _dividerRange = divider;
    _dividerEnabled = enableDiv;

    // Digital divider affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return ksfTkErrOk;
}

bool sfDevAS7331::getSyndTempConversionEnabled(void)
{
    return _tempConvEnabled;
}

sfTkError_t sfDevAS7331::setSyndTempConversionEnabled(const bool &isEnabled)
{
    sfe_as7331_reg_cfg_creg2_t creg2;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg2(creg2);
    if (ksfTkErrOk != result)
        return result;

    creg2.en_tm = isEnabled;

    result = setCReg2(creg2);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _tempConvEnabled = isEnabled;

    return ksfTkErrOk;
}

bool sfDevAS7331::getIndexMode(void)
{
    return _indexMode;
}

sfTkError_t sfDevAS7331::setIndexMode(const bool &indexMode)
{
    sfe_as7331_reg_cfg_optreg_t optreg;

    // Standard read-modify-write sequence.
    sfTkError_t result = getOptIndex(optreg);
    if (ksfTkErrOk != result)
        return result;

    optreg.init_idx = indexMode;

    result = setOptIndex(optreg);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _indexMode = indexMode;

    return ksfTkErrOk;
}

uint8_t sfDevAS7331::getBreakTime(void)
{
    return _breakTime;
}

sfTkError_t sfDevAS7331::setBreakTime(const uint8_t &breakTime)
{
    uint8_t breakreg;

    // Standard read-modify-write sequence.
    sfTkError_t result = getBreak(breakreg);
    if (ksfTkErrOk != result)
        return result;

    breakreg = breakTime;

    result = setBreak(breakreg);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _breakTime = breakTime;

    return ksfTkErrOk;
}

uint8_t sfDevAS7331::getNumEdges(void)
{
    return _numEdges;
}

sfTkError_t sfDevAS7331::setNumEdges(const uint8_t &numEdges)
{
    uint8_t edgesreg;

    // Standard read-modify-write sequence.
    sfTkError_t result = getEdges(edgesreg);
    if (ksfTkErrOk != result)
        return result;

    edgesreg = numEdges;

    result = setEdges(edgesreg);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _numEdges = numEdges;

    return ksfTkErrOk;
}

bool sfDevAS7331::getPowerDownState(void)
{
    return _powerDownEnableState;
}

sfTkError_t sfDevAS7331::setPowerDownState(const bool &pd)
{
    sfe_as7331_reg_cfg_osr_t osr;

    // Standard read-modify-write sequence.
    sfTkError_t result = getOSR(osr);
    if (ksfTkErrOk != result)
        return result;

    osr.pd = pd;

    result = setOSR(osr);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _powerDownEnableState = pd;

    return ksfTkErrOk;
}

as7331_dev_op_state_t sfDevAS7331::getOperationMode(void)
{
    return _opMode;
}

sfTkError_t sfDevAS7331::setOperationMode(const as7331_dev_op_state_t &opMode)
{
    sfe_as7331_reg_cfg_osr_t osr;

    // Standard read-modify-write sequence.
    sfTkError_t result = getOSR(osr);
    if (ksfTkErrOk != result)
        return result;

    osr.dos = opMode;

    result = setOSR(osr);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _opMode = opMode;

    return ksfTkErrOk;
}

as7331_meas_mode_t sfDevAS7331::getMeasurementMode(void)
{
    return _mmode;
}

sfTkError_t sfDevAS7331::setMeasurementMode(const as7331_meas_mode_t &measMode)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    creg3.mmode = measMode;

    result = setCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _mmode = measMode;

    return ksfTkErrOk;
}

bool sfDevAS7331::getStandbyState(void)
{
    return _standbyState;
}

sfTkError_t sfDevAS7331::setStandbyState(const bool &standby)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfTkError_t result = getCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    creg3.sb = standby;

    result = setCReg3(creg3);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _standbyState = standby;

    return ksfTkErrOk;
}

bool sfDevAS7331::getStartState(void)
{
    return _startState;
}

sfTkError_t sfDevAS7331::setStartState(const bool &startState)
{
    sfe_as7331_reg_cfg_osr_t osr;

    // Standard read-modify-write sequence.
    sfTkError_t result = getOSR(osr);
    if (ksfTkErrOk != result)
        return result;

    osr.ss = startState;

    result = setOSR(osr);
    if (ksfTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _startState = startState;

    return ksfTkErrOk;
}

sfTkError_t sfDevAS7331::getStatus(sfe_as7331_reg_meas_osr_status_t &statusReg)
{
    // Status register is only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return ksfTkErrFail;

    uint16_t statusRaw;

    sfTkError_t result = _theBus->readRegisterWord(kSfeAS7331RegMeasOsrStatus, statusRaw);

    if (ksfTkErrOk != result)
        return result;

    statusReg.word = statusRaw;

    return ksfTkErrOk;
}

sfTkError_t sfDevAS7331::getOSR(sfe_as7331_reg_cfg_osr_t &osrReg)
{
    // OSR is available in both operation modes.
    if (!_theBus)
        return ksfTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgOsr, osrReg.byte);
}

sfTkError_t sfDevAS7331::setOSR(const sfe_as7331_reg_cfg_osr_t &osrReg)
{
    // OSR is available in both operation modes.
    if (!_theBus)
        return ksfTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgOsr, osrReg.byte);
}

sfTkError_t sfDevAS7331::getCReg1(sfe_as7331_reg_cfg_creg1_t &creg1)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgCreg1, creg1.byte);
}

sfTkError_t sfDevAS7331::setCReg1(const sfe_as7331_reg_cfg_creg1_t &creg1)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgCreg1, creg1.byte);
}

sfTkError_t sfDevAS7331::getCReg2(sfe_as7331_reg_cfg_creg2_t &creg2)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgCreg2, creg2.byte);
}

sfTkError_t sfDevAS7331::setCReg2(const sfe_as7331_reg_cfg_creg2_t &creg2)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgCreg2, creg2.byte);
}

sfTkError_t sfDevAS7331::getCReg3(sfe_as7331_reg_cfg_creg3_t &creg3)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgCreg3, creg3.byte);
}

sfTkError_t sfDevAS7331::setCReg3(const sfe_as7331_reg_cfg_creg3_t &creg3)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgCreg3, creg3.byte);
}

sfTkError_t sfDevAS7331::getBreak(uint8_t &breakReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgBreak, breakReg);
}

sfTkError_t sfDevAS7331::setBreak(const uint8_t &breakReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgBreak, breakReg);
}

sfTkError_t sfDevAS7331::getEdges(uint8_t &edgesReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgEdges, edgesReg);
}

sfTkError_t sfDevAS7331::setEdges(const uint8_t &edgesReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgEdges, edgesReg);
}

sfTkError_t sfDevAS7331::getOptIndex(sfe_as7331_reg_cfg_optreg_t &optReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgOptReg, optReg.byte);
}

sfTkError_t sfDevAS7331::setOptIndex(const sfe_as7331_reg_cfg_optreg_t &optReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return ksfTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgOptReg, optReg.byte);
}

sfTkError_t sfDevAS7331::readRawUV(const as7331_uv_type &uv_type)
{
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return ksfTkErrFail;

    uint16_t uvRawVal;
    uint8_t regAddress = 0;
    float *retUV = nullptr;
    float *conv = nullptr;
    const float *fsr = nullptr;

    // Set variables based on the UV channel requested.
    switch (uv_type)
    {
    default: // Since it's an enum, you can't miscall this function, so default to A.
    case AS7331_UVA:
        regAddress = kSfeAS7331RegMeasMres1;
        fsr = &_fsrA;
        retUV = &_uva;
        conv = &_conversionA;
        break;
    case AS7331_UVB:
        regAddress = kSfeAS7331RegMeasMres2;
        fsr = &_fsrB;
        retUV = &_uvb;
        conv = &_conversionB;
        break;
    case AS7331_UVC:
        regAddress = kSfeAS7331RegMeasMres3;
        fsr = &_fsrC;
        retUV = &_uvc;
        conv = &_conversionC;
        break;
    }

    sfTkError_t result = _theBus->readRegisterWord(regAddress, uvRawVal);

    if (ksfTkErrOk != result)
        return result;

    // If we're in SYND mode, need to calculate conversion based on the conversion time.
    if (_mmode == MEAS_MODE_SYND)
    {
        result = readOutConv();

        if (ksfTkErrOk != result)
            return result;

        // See datasheet section 7.4 Equation 4.
        //          fsrX
        //      --------------
        //      gain * outConv
        float convFactor = (*fsr) / (((float)_outputConversionTime) * ((float)(1 << (11 - _sensorGain))));

        // Since result is more than 1 byte, need to order it correctly before conversion.
        *retUV = convertRawUVVal(uvRawVal, convFactor);
    }
    else
    {
        // Since result is more than 1 byte, need to order it correctly before conversion.
        *retUV = convertRawUVVal(uvRawVal, *conv);
    }

    return ksfTkErrOk;
}

float sfDevAS7331::convertRawTempToTempC(const uint16_t &inputVal)
{
    // T_chip = TEMP*0.05 - 66.9
    // EX: TEMP=0x922 aka TEMP=0d2338, returns 50.0
    return ((float)(inputVal) * 0.05f) - 66.9f;
}

float sfDevAS7331::convertRawUVVal(const uint16_t &rawVal, const float &convFactor)
{
    // If the divider is enabled, then we need to include the division factor.
    float divFactor = _dividerEnabled ? (float)(1 << (1 + _dividerRange)) : 1.0f;

    return ((float)rawVal) * divFactor * convFactor;
}

void sfDevAS7331::calculateConversionFactors(void)
{
    // See datasheet section 7.4 Equation 3.
    //              fsrX
    //      -------------------
    //      gain * tconv * cclk
    // Calculates a conversion factor based on the sensor gain, conversion time, and conversion clock frequency.
    // Only valid for CONT, CMD, SYNS modes.
    float convFactor = 1.0f / (((float)(1 << (11 - _sensorGain))) * ((float)(1 << _conversionTime)) *
                               ((float)(1024.0 * (1 << _cclk))));

    _conversionA = _fsrA * convFactor;
    _conversionB = _fsrB * convFactor;
    _conversionC = _fsrC * convFactor;
}

void sfDevAS7331::setDefaultSettings()
{
    _breakTime = 25;              // 25 * 8us = 200us.
    _numEdges = 1;                // 1 edge.
    _readyPinMode = false;        // Push-pull.
    _dividerEnabled = false;      // Predivider disabled.
    _tempConvEnabled = true;      // Temp conversion in synd mode enabled.
    _indexMode = true;            // Repeat start enabled.
    _standbyState = false;        // Not in standby mode.
    _startState = false;          // Not started.
    _powerDownEnableState = true; // Device is powered down.
    _opMode = DEVICE_MODE_CFG;    // Device is in configuration mode.
    _sensorGain = GAIN_2;         // Gain of 2x.
    _cclk = CCLK_1_024_MHZ;       // 1.024 MHz conversion clock
    _mmode = MEAS_MODE_CMD;       // Command/One Shot Mode.
    _conversionTime = TIME_64MS;  // 64 ms conversion time.
    _dividerRange = DIV_2;        // Predivider 2x.
}

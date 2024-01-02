#include "sfeAS7331.h"

bool SfeAS7331Driver::begin(sfeTkIBus *theBus)
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

uint8_t SfeAS7331Driver::getDeviceID(void)
{
    // Nullptr check.
    if (!_theBus)
        return 0;

    // Check sensor's operating mode.
    sfe_as7331_reg_cfg_osr_t osr;
    if (kSTkErrOk != getOSR(osr))
        return 0;

    bool needsToBeChangedBack = false;

    // Change if necessary.
    if (osr.dos != DEVICE_MODE_CFG)
    {
        needsToBeChangedBack = true;

        osr.dos = DEVICE_MODE_CFG;

        if(kSTkErrOk != setOSR(osr))
            return 0;
    }
    
    uint8_t devID;

    // Read the device ID register, if it errors then return 0.
    if (kSTkErrOk != _theBus->readRegisterByte(kSfeAS7331RegCfgAgen, devID))
        return 0;

    // If we changed it at first, change it back.
    if(needsToBeChangedBack)
    {
        osr.dos = DEVICE_MODE_MEAS;

        if(kSTkErrOk != setOSR(osr))
            return 0;
    }

    return devID;
}

void SfeAS7331Driver::setCommunicationBus(sfeTkIBus *theBus)
{
    _theBus = theBus;
}

bool SfeAS7331Driver::runDefaultSetup(const bool &runSoftReset)
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
        if (kSTkErrOk != getOSR(osr))
            return false;

        // Change operation mode if necessary.
        if (osr.dos != DEVICE_MODE_CFG)
        {
            osr.dos = DEVICE_MODE_CFG;

            if(kSTkErrOk != setOSR(osr))
                return false;
        }

        // Read all the configuration registers in.
        uint8_t regs[6];

        size_t nRead = 0;
        sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegCfgCreg1, regs, 6U, nRead);
        if (nRead != 6 || result != kSTkErrOk)
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
        if (kSTkErrOk != setOSR(osr))
            return false;

        // Assign the registers to a byte array for writing to the device.
        regs[0] = creg1.byte;
        regs[1] = creg2.byte;
        regs[2] = creg3.byte;
        regs[3] = breakreg;
        regs[4] = edgesreg;
        regs[5] = optreg.byte;

        // Write the bytes to the sensor, ensuring the device matches local settings.
        if (kSTkErrOk != _theBus->writeRegisterRegion(kSfeAS7331RegCfgCreg1, regs, 6U))
            return false;
    }

    // Calculate new conversion factors to make sure they match the current settings.
    calculateConversionFactors();

    return true;
}

bool SfeAS7331Driver::prepareMeasurement(const as7331_meas_mode_t &measMode, const bool &startMeasure)
{
    // If the device is currently in power down mode.
    if (_powerDownEnableState)
    {
        if (kSTkErrOk != setPowerDownState(false)) // Set the device to be powered up.
            return false;
    }
    
    // If the device is currently in standby mode.
    if (_standbyState)
    {
        if (kSTkErrOk != setStandbyState(false)) // Set the device to not be in standby.
            return false;
    }

    // If the device's currently configured measurement mode is different than the desired mode.
    // This is just to reduce unnecessary calls.
    if (_mmode != measMode)
    {
        if (kSTkErrOk != setMeasurementMode(measMode)) // Set the new mode.
            return false;
    }

    // If the device is in start state and we don't want it to be.
    // This prevents starting automatically in the next step.
    if (_startState && !startMeasure)
    {
        if (kSTkErrOk != setStartState(false)) // Stop it from running automatically.
            return false;
    }

    // Now that we've configured ourselves to measure properly, move the device op mode to measure.
    if (kSTkErrOk != setOperationMode(DEVICE_MODE_MEAS))
        return false;

    // If the device is supposed to be started automatically and isn't configured that way.
    if (!_startState && startMeasure)
    {
        if (kSTkErrOk != setStartState(true)) // Set the device to being measuring.
            return false;
    }

    return true;
}

bool SfeAS7331Driver::reset(void)
{
    // OSR is available in all modes, no need to change modes.

    // Standard read-modify-write sequence.
    sfe_as7331_reg_cfg_osr_t osr;

    if (kSTkErrOk != getOSR(osr))
        return false;

    // Set software reset bit.
    osr.sw_res = 1;

    if (kSTkErrOk != setOSR(osr))
        return false;

    // Set internal variables back to defaults.
    setDefaultSettings();

    return true;
}

sfeTkError_t SfeAS7331Driver::readTemp(void)
{
    // Temperature is only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    uint16_t tempRaw;

    // Read in the raw value.
    sfeTkError_t result = _theBus->readRegisterWord(kSfeAS7331RegMeasTemp, tempRaw);

    if (kSTkErrOk != result)
        return result;

    // Since temperature is more than 1 byte, need to order it correctly before conversion.
    _temperature = convertRawTempToTempC(tempRaw);

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::readUVA(void)
{
    return readRawUV(AS7331_UVA);
}

sfeTkError_t SfeAS7331Driver::readUVB(void)
{
    return readRawUV(AS7331_UVB);
}

sfeTkError_t SfeAS7331Driver::readUVC(void)
{
    return readRawUV(AS7331_UVC);
}

sfeTkError_t SfeAS7331Driver::readAllUV(void)
{
    // UV results are only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    uint8_t dataRaw[6];

    // Read in the raw data from the results registers.
    size_t nRead = 0;

    sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasMres1, dataRaw, 6U, nRead);

    if (nRead != 6 || result != kSTkErrOk)
        return result;

    // If we're in SYND mode, need to calculate conversion based on the conversion time.
    if (_mmode == MEAS_MODE_SYND)
    {
        result = readOutConv();
        if (kSTkErrOk != result)
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

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::readAll(void)
{
    // Results are only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    uint8_t dataRaw[8];

    size_t nRead = 0;
    sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasTemp, dataRaw, 8U, nRead);

    if (nRead != 8 || result != kSTkErrOk)
        return result;

    result = readOutConv();
    if (kSTkErrOk != result)
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

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::readOutConv(void)
{
    // Results are only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    uint8_t tconvRaw[4];

    size_t nRead;

    sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasOutConvL, tconvRaw, 4U, nRead);

    if (nRead != 4 || result != kSTkErrOk)
        return result;

    // Since result is more than 1 byte, need to order it correctly.
    _outputConversionTime = (uint32_t)(((uint32_t)tconvRaw[3] << 24) | ((uint32_t)tconvRaw[2] << 16) |
                                       ((uint32_t)tconvRaw[1] << 8) | tconvRaw[0]);

    return kSTkErrOk;
}

float SfeAS7331Driver::getUVA(void)
{
    return _uva;
}

float SfeAS7331Driver::getUVB(void)
{
    return _uvb;
}

float SfeAS7331Driver::getUVC(void)
{
    return _uvc;
}

float SfeAS7331Driver::getTemp(void)
{
    return _temperature;
}

uint32_t SfeAS7331Driver::getOutConv(void)
{
    return _outputConversionTime;
}

as7331_gain_t SfeAS7331Driver::getGainRaw(void)
{
    return _sensorGain;
}

uint16_t SfeAS7331Driver::getGainValue(void)
{
    return (1 << (11 - _sensorGain));
}

sfeTkError_t SfeAS7331Driver::setGain(const as7331_gain_t &gain)
{
    sfe_as7331_reg_cfg_creg1_t creg1;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    creg1.gain = gain;

    result = setCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _sensorGain = gain;

    // Gain affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return kSTkErrOk;
}

as7331_conv_clk_freq_t SfeAS7331Driver::getCClkRaw(void)
{
    return _cclk;
}

uint16_t SfeAS7331Driver::getCClkKHz(void)
{
    return 1024*(1 << _cclk);
}

sfeTkError_t SfeAS7331Driver::setCClk(const as7331_conv_clk_freq_t &cclk)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.cclk = cclk;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _cclk = cclk;

    // CClk affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return kSTkErrOk;
}

as7331_conv_time_t SfeAS7331Driver::getConversionTimeRaw(void)
{
    return _conversionTime;
}

uint16_t SfeAS7331Driver::getConversionTimeMillis(void)
{
    return (1 << _conversionTime);
}

sfeTkError_t SfeAS7331Driver::setConversionTime(const as7331_conv_time_t &convTime)
{
    sfe_as7331_reg_cfg_creg1_t creg1;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    creg1.time = convTime;

    result = setCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _conversionTime = convTime;

    // Conversion Time affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return kSTkErrOk;
}

bool SfeAS7331Driver::getReadyPinMode(void)
{
    return _readyPinMode;
}

sfeTkError_t SfeAS7331Driver::setReadyPinMode(const bool &pinMode)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.rdyod = pinMode;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _readyPinMode = pinMode;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getDigitalDividerEnabled(void)
{
    return _dividerEnabled;
}

sfeTkError_t SfeAS7331Driver::setDigitalDividerEnabled(const bool &isEnabled)
{
    if (_dividerEnabled == isEnabled)
        return kSTkErrOk;

    sfe_as7331_reg_cfg_creg2_t creg2;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    creg2.en_div = isEnabled;

    result = setCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _dividerEnabled = isEnabled;

    // Digital divider affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return kSTkErrOk;
}

as7331_divider_val_t SfeAS7331Driver::getDigitalDividerRange(void)
{
    return _dividerRange;
}

sfeTkError_t SfeAS7331Driver::setDigitalDividerRange(const as7331_divider_val_t &divider, const bool &enableDiv)
{
    sfe_as7331_reg_cfg_creg2_t creg2;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    creg2.div = divider;
    creg2.en_div = enableDiv;

    result = setCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _dividerRange = divider;
    _dividerEnabled = enableDiv;

    // Digital divider affects the conversion factors, so calculate new ones with the updated values.
    calculateConversionFactors();

    return kSTkErrOk;
}

bool SfeAS7331Driver::getSyndTempConversionEnabled(void)
{
    return _tempConvEnabled;
}

sfeTkError_t SfeAS7331Driver::setSyndTempConversionEnabled(const bool &isEnabled)
{
    sfe_as7331_reg_cfg_creg2_t creg2;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    creg2.en_tm = isEnabled;

    result = setCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _tempConvEnabled = isEnabled;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getIndexMode(void)
{
    return _indexMode;
}

sfeTkError_t SfeAS7331Driver::setIndexMode(const bool &indexMode)
{
    sfe_as7331_reg_cfg_optreg_t optreg;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getOptIndex(optreg);
    if (kSTkErrOk != result)
        return result;

    optreg.init_idx = indexMode;

    result = setOptIndex(optreg);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _indexMode = indexMode;

    return kSTkErrOk;
}

uint8_t SfeAS7331Driver::getBreakTime(void)
{
    return _breakTime;
}

sfeTkError_t SfeAS7331Driver::setBreakTime(const uint8_t &breakTime)
{
    uint8_t breakreg;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getBreak(breakreg);
    if (kSTkErrOk != result)
        return result;

    breakreg = breakTime;

    result = setBreak(breakreg);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _breakTime = breakTime;

    return kSTkErrOk;
}

uint8_t SfeAS7331Driver::getNumEdges(void)
{
    return _numEdges;
}

sfeTkError_t SfeAS7331Driver::setNumEdges(const uint8_t &numEdges)
{
    uint8_t edgesreg;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getEdges(edgesreg);
    if (kSTkErrOk != result)
        return result;

    edgesreg = numEdges;

    result = setEdges(edgesreg);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _numEdges = numEdges;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getPowerDownState(void)
{
    return _powerDownEnableState;
}

sfeTkError_t SfeAS7331Driver::setPowerDownState(const bool &pd)
{
    sfe_as7331_reg_cfg_osr_t osr;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getOSR(osr);
    if (kSTkErrOk != result)
        return result;

    osr.pd = pd;

    result = setOSR(osr);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _powerDownEnableState = pd;

    return kSTkErrOk;
}

as7331_dev_op_state_t SfeAS7331Driver::getOperationMode(void)
{
    return _opMode;
}

sfeTkError_t SfeAS7331Driver::setOperationMode(const as7331_dev_op_state_t &opMode)
{
    sfe_as7331_reg_cfg_osr_t osr;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getOSR(osr);
    if (kSTkErrOk != result)
        return result;

    osr.dos = opMode;

    result = setOSR(osr);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _opMode = opMode;

    return kSTkErrOk;
}

as7331_meas_mode_t SfeAS7331Driver::getMeasurementMode(void)
{
    return _mmode;
}

sfeTkError_t SfeAS7331Driver::setMeasurementMode(const as7331_meas_mode_t &measMode)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.mmode = measMode;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _mmode = measMode;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getStandbyState(void)
{
    return _standbyState;
}

sfeTkError_t SfeAS7331Driver::setStandbyState(const bool &standby)
{
    sfe_as7331_reg_cfg_creg3_t creg3;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.sb = standby;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _standbyState = standby;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getStartState(void)
{
    return _startState;
}

sfeTkError_t SfeAS7331Driver::setStartState(const bool &startState)
{
    sfe_as7331_reg_cfg_osr_t osr;

    // Standard read-modify-write sequence.
    sfeTkError_t result = getOSR(osr);
    if (kSTkErrOk != result)
        return result;

    osr.ss = startState;

    result = setOSR(osr);
    if (kSTkErrOk != result)
        return result;

    // If write is successful, save to internal state.
    _startState = startState;

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::getStatus(sfe_as7331_reg_meas_osr_status_t &statusReg)
{
    // Status register is only available in Measurement mode.
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    uint16_t statusRaw;

    sfeTkError_t result = _theBus->readRegisterWord(kSfeAS7331RegMeasOsrStatus, statusRaw);

    if (kSTkErrOk != result)
        return result;

    statusReg.word = statusRaw;

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::getOSR(sfe_as7331_reg_cfg_osr_t &osrReg)
{
    // OSR is available in both operation modes.
    if (!_theBus)
        return kSTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgOsr, osrReg.byte);
}

sfeTkError_t SfeAS7331Driver::setOSR(const sfe_as7331_reg_cfg_osr_t &osrReg)
{
    // OSR is available in both operation modes.
    if (!_theBus)
        return kSTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgOsr, osrReg.byte);
}

sfeTkError_t SfeAS7331Driver::getCReg1(sfe_as7331_reg_cfg_creg1_t &creg1)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgCreg1, creg1.byte);
}

sfeTkError_t SfeAS7331Driver::setCReg1(const sfe_as7331_reg_cfg_creg1_t &creg1)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgCreg1, creg1.byte);
}

sfeTkError_t SfeAS7331Driver::getCReg2(sfe_as7331_reg_cfg_creg2_t &creg2)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgCreg2, creg2.byte);
}

sfeTkError_t SfeAS7331Driver::setCReg2(const sfe_as7331_reg_cfg_creg2_t &creg2)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgCreg2, creg2.byte);
}

sfeTkError_t SfeAS7331Driver::getCReg3(sfe_as7331_reg_cfg_creg3_t &creg3)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgCreg3, creg3.byte);
}

sfeTkError_t SfeAS7331Driver::setCReg3(const sfe_as7331_reg_cfg_creg3_t &creg3)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgCreg3, creg3.byte);
}

sfeTkError_t SfeAS7331Driver::getBreak(uint8_t &breakReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgBreak, breakReg);
}

sfeTkError_t SfeAS7331Driver::setBreak(const uint8_t &breakReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgBreak, breakReg);
}

sfeTkError_t SfeAS7331Driver::getEdges(uint8_t &edgesReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgEdges, edgesReg);
}

sfeTkError_t SfeAS7331Driver::setEdges(const uint8_t &edgesReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgEdges, edgesReg);
}

sfeTkError_t SfeAS7331Driver::getOptIndex(sfe_as7331_reg_cfg_optreg_t &optReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterByte(kSfeAS7331RegCfgOptReg, optReg.byte);
}

sfeTkError_t SfeAS7331Driver::setOptIndex(const sfe_as7331_reg_cfg_optreg_t &optReg)
{
    // Config registers are only available in Configuration mode.
    if (!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterByte(kSfeAS7331RegCfgOptReg, optReg.byte);
}

sfeTkError_t SfeAS7331Driver::readRawUV(const as7331_uv_type &uv_type)
{
    if (!_theBus || _opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

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

    sfeTkError_t result = _theBus->readRegisterWord(regAddress, uvRawVal);

    if (kSTkErrOk != result)
        return result;

    // If we're in SYND mode, need to calculate conversion based on the conversion time.
    if (_mmode == MEAS_MODE_SYND)
    {
        result = readOutConv();

        if (kSTkErrOk != result)
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

    return kSTkErrOk;
}

float SfeAS7331Driver::convertRawTempToTempC(const uint16_t &inputVal)
{
    // T_chip = TEMP*0.05 - 66.9
    // EX: TEMP=0x922 aka TEMP=0d2338, returns 50.0
    return ((float)(inputVal) * 0.05f) - 66.9f;
}

float SfeAS7331Driver::convertRawUVVal(const uint16_t &rawVal, const float &convFactor)
{
    // If the divider is enabled, then we need to include the division factor.
    float divFactor = _dividerEnabled ? (float)(1 << (1 + _dividerRange)) : 1.0f;

    return ((float)rawVal) * divFactor * convFactor;
}

void SfeAS7331Driver::calculateConversionFactors(void)
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

void SfeAS7331Driver::setDefaultSettings()
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

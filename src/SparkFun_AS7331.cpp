#include "SparkFun_AS7331.h"

bool SfeAS7331Driver::begin(const uint8_t &deviceAddress, sfeTkIBus *theBus)
{
    // Set the internal bus pointer
    setCommunicationBus(theBus);

    // If the address passed in isn't the default, set the new address.
    if(kDefaultAS7331Addr != deviceAddress)
        setDeviceAddress(deviceAddress);

    // Perform a soft reset so that we make sure the device is addressable.
    reset();

    // Get the device setup and ready.
    return runDefaultSetup();
}

uint8_t SfeAS7331Driver::getDeviceID(void)
{
    uint8_t devID;

    // Read the device ID register, if it errors then return 0.
    if (kSTkErrOk != _theBus->readRegisterRegion(kSfeAS7331RegCfgAgen, &devID, 1U))
        return 0;

    return devID;
}

void SfeAS7331Driver::setCommunicationBus(sfeTkIBus *theBus)
{
    _theBus = theBus;
}

void SfeAS7331Driver::setDeviceAddress(const uint8_t &deviceAddress)
{
    _address = deviceAddress;
}

// TODO: Take a look.
bool SfeAS7331Driver::runDefaultSetup(bool runSoftReset)
{
    // Do we need to run a software reset?
    if (runSoftReset)
        reset();
    else
        setDefaultSettings(); // This is performed in the reset() function, don't repeat.

    // Read the OSR register by itself since the offset isn't contiguous with the rest.
    sfe_as7331_reg_cfg_osr_t osr;
    if (kSTkErrOk != getOSR(osr))
        return false;

    // Read all the configuration registers in.
    uint8_t regs[6];
    if (kSTkErrOk != _theBus->readRegisterRegion(kSfeAS7331RegCfgCreg1, regs, 6U))
        return false;

    // Assign the read in bytes to each register's byte union.
    // This allows us to address the individual bits and set them.
    sfe_as7331_reg_cfg_creg1_t creg1 = {.byte = regs[0]};
    sfe_as7331_reg_cfg_creg2_t creg2 = {.byte = regs[1]};
    sfe_as7331_reg_cfg_creg3_t creg3 = {.byte = regs[2]};
    uint8_t breakreg = regs[3];
    uint8_t edgesreg = regs[4];
    sfe_as7331_reg_cfg_optreg_t optreg = {.byte = regs[5]};

    // Here we make sure the local settings match the sensor's settings
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

    // Calculate new conversion factors to make sure they match the current settings.
    calculateConversionFactors();

    return true;
}

// TODO: FIX
bool SfeAS7331Driver::prepareMeasurement(const as7331_meas_mode_t measMode, bool startMeasure)
{
    if(_opMode != DEVICE_MODE_CFG)
        return false;

    if (_powerDownEnableState)
        if (kSTkErrOk != setPowerDownState(false))
            return false;

    if (_mmode != measMode)
    {
        if (kSTkErrOk != setStandbyState(false))
            return false;
        if (kSTkErrOk != setMeasurementMode(measMode))
            return false;
    }

    if (_opMode != DEVICE_MODE_MEAS)
        if (kSTkErrOk != setOperationMode(DEVICE_MODE_MEAS))
            return false;

    if (startMeasure)
        if (kSTkErrOk != setStartState(true))
            return false;

    return true;
}

bool SfeAS7331Driver::reset(void)
{
    sfe_as7331_reg_cfg_osr_t osr;

    if (kSTkErrOk != getOSR(osr))
        return false;

    osr.sw_res = 1;

    if (kSTkErrOk != setOSR(osr))
        return false;

    setDefaultSettings();

    return true;
}

sfeTkError_t SfeAS7331Driver::readTemp(void)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    uint8_t tempRaw[2];

    sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasTemp, tempRaw, 2U);

    if (kSTkErrOk != result)
        return result;

    _temperature = convertRawTempToTempC((uint16_t)tempRaw[1] << 8 | tempRaw[0]);

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::readUVA(void)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    return readRawUV(AS7331_UVA);
}

sfeTkError_t SfeAS7331Driver::readUVB(void)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    return readRawUV(AS7331_UVB);
}

sfeTkError_t SfeAS7331Driver::readUVC(void)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    return readRawUV(AS7331_UVC);
}

sfeTkError_t SfeAS7331Driver::readAllUV(void)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;

    uint8_t dataRaw[6];

    sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasMres1, dataRaw, 6U);

    if (kSTkErrOk != result)
        return result;

    if (_mmode == MEAS_MODE_SYND)
    {
        result = readOutConv();

        if (kSTkErrOk != result)
            return result;

        float convFactor = 1.0f / (((float)_outputConversionTime) * ((float)(1 << (11 - _sensorGain))));

        _uva = convertRawUVVal(((uint16_t)dataRaw[1] << 8 | dataRaw[0]) * _fsrA, convFactor);
        _uvb = convertRawUVVal(((uint16_t)dataRaw[3] << 8 | dataRaw[2]) * _fsrB, convFactor);
        _uvc = convertRawUVVal(((uint16_t)dataRaw[5] << 8 | dataRaw[4]) * _fsrC, convFactor);
    }
    else
    {
        _uva = convertRawUVVal((uint16_t)dataRaw[1] << 8 | dataRaw[0], _conversionA);
        _uvb = convertRawUVVal((uint16_t)dataRaw[3] << 8 | dataRaw[2], _conversionB);
        _uvc = convertRawUVVal((uint16_t)dataRaw[5] << 8 | dataRaw[4], _conversionC);
    }

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::readAll(void)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;
        
    uint8_t dataRaw[8];

    sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasTemp, dataRaw, 8U);

    if (kSTkErrOk != result)
        return result;

    result = readOutConv();

    if (kSTkErrOk != result)
        return result;

    if (_mmode == MEAS_MODE_SYND)
    {
        float convFactor = 1.0f / (((float)_outputConversionTime) * ((float)(1 << (11 - _sensorGain))));

        _uva = convertRawUVVal(((uint16_t)dataRaw[3] << 8 | dataRaw[2]) * _fsrA, convFactor);
        _uvb = convertRawUVVal(((uint16_t)dataRaw[5] << 8 | dataRaw[4]) * _fsrB, convFactor);
        _uvc = convertRawUVVal(((uint16_t)dataRaw[7] << 8 | dataRaw[6]) * _fsrC, convFactor);
    }
    else
    {
        _uva = convertRawUVVal((uint16_t)dataRaw[3] << 8 | dataRaw[2], _conversionA);
        _uvb = convertRawUVVal((uint16_t)dataRaw[5] << 8 | dataRaw[4], _conversionB);
        _uvc = convertRawUVVal((uint16_t)dataRaw[7] << 8 | dataRaw[6], _conversionC);
    }

    _temperature = convertRawTempToTempC((uint16_t)dataRaw[1] << 8 | dataRaw[0]);

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::readOutConv(void)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;
        
    uint8_t tconvRaw[4];

    sfeTkError_t result = _theBus->readRegisterRegion(kSfeAS7331RegMeasOutConvL, tconvRaw, 4U);

    if (kSTkErrOk != result)
        return result;

    _outputConversionTime = (uint32_t)(((uint32_t)tconvRaw[3] << 24) | 
                                       ((uint32_t)tconvRaw[2] << 16) |
                                       ((uint32_t)tconvRaw[1] << 8)  | 
                                       tconvRaw[0]);

    return kSTkErrOk;
}

as7331_gain_t SfeAS7331Driver::getGain(void)
{
    return _sensorGain;
}

sfeTkError_t SfeAS7331Driver::setGain(const as7331_gain_t &gain)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg1_t creg1;

    result = getCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    creg1.gain = gain;

    result = setCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    _sensorGain = gain;

    calculateConversionFactors();

    return kSTkErrOk;
}

as7331_conv_clk_freq_t SfeAS7331Driver::getCClk(void)
{
    return _cclk;
}

sfeTkError_t SfeAS7331Driver::setCClk(const as7331_conv_clk_freq_t &cclk)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.cclk = cclk;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    _cclk = cclk;

    calculateConversionFactors();

    return kSTkErrOk;
}

as7331_conv_time_t SfeAS7331Driver::getConversionTime(void)
{
    return _conversionTime;
}

sfeTkError_t SfeAS7331Driver::setConversionTime(const as7331_conv_time_t &convTime)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg1_t creg1;

    result = getCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    creg1.time = convTime;

    result = setCReg1(creg1);
    if (kSTkErrOk != result)
        return result;

    _conversionTime = convTime;

    calculateConversionFactors();

    return kSTkErrOk;
}

bool SfeAS7331Driver::getReadyPinMode(void)
{
    return _readyPinMode;
}

sfeTkError_t SfeAS7331Driver::setReadyPinMode(const bool &pinMode)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.rdyod = pinMode;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    _readyPinMode = pinMode;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getDigitalDividerEnabled(void)
{
    return _dividerEnabled;
}

// TODO: FIX
sfeTkError_t SfeAS7331Driver::setDigitalDividerEnabled(const bool &isEnabled)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    // TODO: REDO THIS
    if (_dividerEnabled == isEnabled)
        return kSTkErrOk;

    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg2_t creg2;

    result = getCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    creg2.en_div = isEnabled;

    result = setCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    _dividerEnabled = isEnabled;

    if (isEnabled)
        calculateConversionFactors();

    return kSTkErrOk;
}

as7331_divider_val_t SfeAS7331Driver::getDigitalDividerRange(void)
{
    return _dividerRange;
}

// TODO: FIX
sfeTkError_t SfeAS7331Driver::setDigitalDividerRange(const as7331_divider_val_t &divider, const bool &setEnableDiv)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    // TODO: REDO THIS
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg2_t creg2;

    result = getCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    creg2.div = divider;

    result = setCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    _dividerRange = divider;

    if (setEnableDiv)
        result = setDigitalDividerEnabled(true);
    else
        calculateConversionFactors();

    return kSTkErrOk;
}

bool SfeAS7331Driver::getSyndTempConversionEnabled(void)
{
    return _tempConvEnabled;
}

sfeTkError_t SfeAS7331Driver::setSyndTempConversionEnabled(const bool &isEnabled)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg2_t creg2;

    result = getCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    creg2.en_tm = isEnabled;

    result = setCReg2(creg2);
    if (kSTkErrOk != result)
        return result;

    _tempConvEnabled = isEnabled;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getIndexMode(void)
{
    return _indexMode;
}

sfeTkError_t SfeAS7331Driver::setIndexMode(const bool &indexMode)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_optreg_t optreg;

    result = getOptIndex(optreg);
    if (kSTkErrOk != result)
        return result;

    optreg.init_idx = indexMode;

    result = setOptIndex(optreg);
    if (kSTkErrOk != result)
        return result;

    _indexMode = indexMode;

    return kSTkErrOk;
}

uint8_t SfeAS7331Driver::getBreakTime(void)
{
    return _breakTime;
}

sfeTkError_t SfeAS7331Driver::setBreakTime(const uint8_t &breakTime)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    uint8_t breakreg;

    result = getBreak(breakreg);
    if (kSTkErrOk != result)
        return result;

    breakreg = breakTime;

    result = setBreak(breakreg);
    if (kSTkErrOk != result)
        return result;

    _breakTime = breakTime;

    return kSTkErrOk;
}

uint8_t SfeAS7331Driver::getNumEdges(void)
{
    return _numEdges;
}

sfeTkError_t SfeAS7331Driver::setNumEdges(const uint8_t &numEdges)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    uint8_t edgesreg;

    result = getEdges(edgesreg);
    if (kSTkErrOk != result)
        return result;

    edgesreg = numEdges;

    result = setEdges(edgesreg);
    if (kSTkErrOk != result)
        return result;

    _numEdges = numEdges;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getPowerDownState(void)
{
    return _powerDownEnableState;
}

sfeTkError_t SfeAS7331Driver::setPowerDownState(const bool &pd)
{
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_osr_t osr;

    result = getOSR(osr);
    if (kSTkErrOk != result)
        return result;

    osr.pd = pd;

    result = setOSR(osr);
    if (kSTkErrOk != result)
        return result;

    _powerDownEnableState = pd;

    return kSTkErrOk;
}

as7331_device_op_state_t SfeAS7331Driver::getOperationMode(void)
{
    return _opMode;
}

sfeTkError_t SfeAS7331Driver::setOperationMode(const as7331_device_op_state_t &opMode)
{
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_osr_t osr;

    result = getOSR(osr);
    if (kSTkErrOk != result)
        return result;

    osr.dos = opMode;

    result = setOSR(osr);
    if (kSTkErrOk != result)
        return result;

    _opMode = opMode;

    return kSTkErrOk;
}

as7331_meas_mode_t SfeAS7331Driver::getMeasurementMode(void)
{
    return _mmode;
}

sfeTkError_t SfeAS7331Driver::setMeasurementMode(const as7331_meas_mode_t &measMode)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.mmode = measMode;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    _mmode = measMode;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getStandbyState(void)
{
    return _standbyState;
}

sfeTkError_t SfeAS7331Driver::setStandbyState(const bool &standby)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    creg3.sb = standby;

    result = setCReg3(creg3);
    if (kSTkErrOk != result)
        return result;

    _standbyState = standby;

    return kSTkErrOk;
}

bool SfeAS7331Driver::getStartState(void)
{
    return _startState;
}

sfeTkError_t SfeAS7331Driver::setStartState(const bool &startState)
{
    sfeTkError_t result = kSTkErrOk;

    sfe_as7331_reg_cfg_osr_t osr;

    result = getOSR(osr);
    if (kSTkErrOk != result)
        return result;

    osr.ss = startState;

    result = setOSR(osr);
    if (kSTkErrOk != result)
        return result;

    _startState = startState;

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::getStatus(sfe_as7331_reg_meas_osr_status_t &statusReg)
{
    if(_opMode != DEVICE_MODE_MEAS)
        return kSTkErrFail;
        
    sfeTkError_t result = kSTkErrOk;
    uint8_t statusRaw[2];

    result = _theBus->readRegisterRegion(kSfeAS7331RegMeasOsrStatus, statusRaw, 2U);

    if (kSTkErrOk != result)
        return result;

    // Shift MSB to top of 16-bit register, then OR with LSB.
    statusReg.word = (uint16_t)statusRaw[1] << 8 | statusRaw[0];

    return kSTkErrOk;
}

sfeTkError_t SfeAS7331Driver::getOSR(sfe_as7331_reg_cfg_osr_t &osrReg)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->readRegisterRegion(kSfeAS7331RegCfgOsr, &osrReg.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::setOSR(const sfe_as7331_reg_cfg_osr_t &osrReg)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->writeRegisterRegion(kSfeAS7331RegCfgOsr, &osrReg.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::getCReg1(sfe_as7331_reg_cfg_creg1_t &creg1)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->readRegisterRegion(kSfeAS7331RegCfgCreg1, &creg1.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::setCReg1(const sfe_as7331_reg_cfg_creg1_t &creg1)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->writeRegisterRegion(kSfeAS7331RegCfgCreg1, &creg1.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::getCReg2(sfe_as7331_reg_cfg_creg2_t &creg2)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->readRegisterRegion(kSfeAS7331RegCfgCreg2, &creg2.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::setCReg2(const sfe_as7331_reg_cfg_creg2_t &creg2)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->writeRegisterRegion(kSfeAS7331RegCfgCreg2, &creg2.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::getCReg3(sfe_as7331_reg_cfg_creg3_t &creg3)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->readRegisterRegion(kSfeAS7331RegCfgCreg3, &creg3.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::setCReg3(const sfe_as7331_reg_cfg_creg3_t &creg3)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->writeRegisterRegion(kSfeAS7331RegCfgCreg3, &creg3.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::getBreak(uint8_t &breakReg)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->readRegisterRegion(kSfeAS7331RegCfgBreak, &breakReg, 1U);
}

sfeTkError_t SfeAS7331Driver::setBreak(const uint8_t &breakReg)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;
        
    return _theBus->writeRegisterRegion(kSfeAS7331RegCfgBreak, &breakReg, 1U);
}

sfeTkError_t SfeAS7331Driver::getEdges(uint8_t &edgesReg)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterRegion(kSfeAS7331RegCfgEdges, &edgesReg, 1U);
}

sfeTkError_t SfeAS7331Driver::setEdges(const uint8_t &edgesReg)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterRegion(kSfeAS7331RegCfgEdges, &edgesReg, 1U);
}

sfeTkError_t SfeAS7331Driver::getOptIndex(sfe_as7331_reg_cfg_optreg_t &optReg)
{
    if(_opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->readRegisterRegion(kSfeAS7331RegCfgOptReg, &optReg.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::setOptIndex(const sfe_as7331_reg_cfg_optreg_t &optReg)
{
    if(!_theBus || _opMode != DEVICE_MODE_CFG)
        return kSTkErrFail;

    return _theBus->writeRegisterRegion(kSfeAS7331RegCfgOptReg, &optReg.byte, 1U);
}

sfeTkError_t SfeAS7331Driver::readRawUV(const as7331_uv_type &uv_type)
{
    uint8_t uvRawVal[2];
    uint8_t address = 0;
    float *retUV = nullptr;
    float *conv = nullptr;
    const float *fsr = nullptr;

    switch (uv_type)
    {
    case AS7331_UVA:
    default: // Since it's an enum, you can't miscall this function, so default to A.
        address = kSfeAS7331RegMeasMres1;
        fsr = &_fsrA;
        retUV = &_uva;
        if (_mmode != MEAS_MODE_SYND)
        {
            conv = &_conversionA;
        }
        break;
    case AS7331_UVB:
        address = kSfeAS7331RegMeasMres2;
        fsr = &_fsrB;
        retUV = &_uvb;
        if (_mmode != MEAS_MODE_SYND)
        {
            conv = &_conversionB;
        }
        break;
    case AS7331_UVC:
        address = kSfeAS7331RegMeasMres3;
        fsr = &_fsrC;
        retUV = &_uvc;
        if (_mmode != MEAS_MODE_SYND)
        {
            conv = &_conversionC;
        }
        break;
    }

    sfeTkError_t result = _theBus->readRegisterRegion(address, uvRawVal, 2U);

    if (kSTkErrOk != result)
        return result;

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

        *retUV = convertRawUVVal((uint16_t)uvRawVal[1] << 8 | uvRawVal[0], convFactor);
    }
    else
    {
        *retUV = convertRawUVVal((uint16_t)uvRawVal[1] << 8 | uvRawVal[0], *conv);
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
    _breakTime = 25; // 25 * 8us = 200us.
    _numEdges = 1; // 1 edge.
    _readyPinMode = false; // Push-pull.
    _dividerEnabled = false; // Predivider disabled.
    _tempConvEnabled = true; // Temp conversion in synd mode enabled.
    _indexMode = true; // Repeat start enabled.
    _standbyState = false; // Not in standby mode.
    _startState = false; // Not started.
    _powerDownEnableState = true; // Device is powered down.
    _opMode = DEVICE_MODE_CFG; // Device is in configuration mode.
    _sensorGain = GAIN_2; // Gain of 2x.
    _cclk = CCLK_1_024_MHZ; // 1.024 MHz conversion clock
    _mmode = MEAS_MODE_CMD; // Command/One Shot Mode.
    _conversionTime = TIME_64MS; // 64 ms conversion time.
    _dividerRange = DIV_2; // Predivider 2x.
}
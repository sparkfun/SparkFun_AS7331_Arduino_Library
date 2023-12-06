#include "SparkFun_AS7331.h"

// TODO: FIX
bool SfeAS7331Driver::begin(sfeBusDevice *theBus, sfeBusDeviceSettings *devSettings, const uint8_t deviceAddress)
{
    setCommunicationBus(theBus, devSettings);
    return begin(deviceAddress);
}

// TODO: FIX
bool SfeAS7331Driver::begin(sfeBusDevice *theBus, uint8_t deviceAddress)
{
    setCommunicationBus(theBus);
    return begin(deviceAddress);
}

// TODO: FIX
bool SfeAS7331Driver::begin(uint8_t deviceAddress)
{
    if (!_sfeBus)
        setCommunicationBus();

    setDeviceAddress(deviceAddress);

    _sfeBus->begin();

    return init();
}

// TODO: FIX
bool SfeAS7331Driver::init(bool runSetup)
{
    if (SFE_BUS_OK != _sfeBus->ping(_devSettings))
        return false;

    reset();

    if (!isConnected())
        return false;

    if (runSetup)
        return runDefaultSetup();

    return true;
}

// bool SfeAS7331Driver::isConnected(void)
// {
//     return (AS7331_DEFAULT_DEV_ID == getDeviceID());
// }

uint8_t SfeAS7331Driver::getDeviceID(void)
{
    uint8_t devID;

    if (SFE_BUS_OK != _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_AGEN, &devID, 1U))
        return 0;

    return devID;
}

// TODO: FIX
bool SfeAS7331Driver::runDefaultSetup(bool runSoftReset = false)
{
    if (runSoftReset)
        return reset();
    else
    {
        _state = stateDefault;
        _config = configDefault;
        measures = measuresDefault;
    }

    sfe_as7331_reg_cfg_osr_t osr;
    if (SFE_BUS_OK != getOSR(&osr))
        return false;

    uint8_t regs[6];
    if (SFE_BUS_OK != readRegister(SFE_AS7331_REGISTER_CFG_CREG1, regs, 6U))
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

    if (SFE_BUS_OK != setOSR(&osr))
        return false;

    regs[0] = creg1.byte;
    regs[1] = creg2.byte;
    regs[2] = creg3.byte;
    regs[3] = breakreg;
    regs[4] = edgesreg;
    regs[5] = optreg.byte;

    if (SFE_BUS_OK != writeRegister(SFE_AS7331_REGISTER_CFG_CREG1, regs, 6U))
        return false;

    calculateConversionFactors();

    return true;
}

// TODO: FIX
bool SfeAS7331Driver::prepareMeasurement(const as7331_meas_mode_t measMode = MEAS_MODE_CONT, bool startMeasure = false)
{
    if (_state.pd == POWER_DOWN_ENABLE)
        if (SFE_BUS_OK != setPowerState(POWER_DOWN_DISABLE))
            return false;

    if (_state.mmode != measMode)
    {
        if (SFE_BUS_OK != setStandbyMode(STANDBY_DISABLED))
            return false;
        if (SFE_BUS_OK != setMeasurementMode(measMode))
            return false;
    }

    if (_state.opMode != DEVICE_MODE_MEAS)
        if (SFE_BUS_OK != setOperationMode(DEVICE_MODE_MEAS))
            return false;

    if (startMeasure)
        if (SFE_BUS_OK != setStartStateMode(START_STATE_ENABLED))
            return false;

    return true;
}

// TODO: FIX
int8_t SfeAS7331Driver::setCommunicationBus(sfeBusDevice *theBus, sfeBusDeviceSettings *deviceSettings)
{
    int8_t result = setCommunicationBus(theBus);
    if (SFE_BUS_OK != result)
        return result;

    return setCommunicationDevSettings(deviceSettings);
}

// TODO: FIX
int8_t SfeAS7331Driver::setCommunicationBus(sfeBusDevice *theBus = nullptr)
{
    if (theBus == nullptr)
        theBus = new sfeBusDevice();

    if (!theBus)
        return SFE_BUS_E_NULL_PTR;

    _sfeBus = theBus;

    return SFE_BUS_OK;
}

// TODO: FIX
int8_t SfeAS7331Driver::setCommunicationDevSettings(sfeBusDeviceSettings *deviceSettings = nullptr)
{
    if (deviceSettings == nullptr)
        deviceSettings = new sfeBusDeviceSettings();

    if (!deviceSettings)
        return SFE_BUS_E_NULL_DEV_SETTINGS;

    _devSettings = deviceSettings;

    return SFE_BUS_OK;
}

// TODO: FIX
int8_t SfeAS7331Driver::setDeviceAddress(const uint8_t deviceAddress)
{
    int8_t result = SFE_BUS_OK;

    if (!_devSettings)
        result = setCommunicationDevSettings();

    if (SFE_BUS_OK != result)
        return result;

    _devSettings->devAddr = deviceAddress;

    return SFE_BUS_OK;
}

bool SfeAS7331Driver::reset(void)
{
    sfe_as7331_reg_cfg_osr_t osr;

    if (SFE_BUS_OK != getOSR(osr))
        return false;

    osr.sw_res = 1;

    if (SFE_BUS_OK != setOSR(osr))
        return false;

    setDefaultSettings();

    return true;
}

int8_t SfeAS7331Driver::readTemp(void)
{
    uint8_t tempRaw[2];

    int8_t result = _theBus->readRegisterRegion(SFE_AS7331_REGISTER_MEAS_TEMP, tempRaw, 2U);

    if (SFE_BUS_OK != result)
        return result;

    _temperature = convertRawTempToTempC((uint16_t)tempRaw[1] << 8 | tempRaw[0]);

    return SFE_BUS_OK;
}

int8_t SfeAS7331Driver::readUVA(void)
{
    return readRawUV(AS7331_UVA);
}

int8_t SfeAS7331Driver::readUVB(void)
{
    return readRawUV(AS7331_UVB);
}

int8_t SfeAS7331Driver::readUVC(void)
{
    return readRawUV(AS7331_UVC);
}

int8_t SfeAS7331Driver::readAllUV(void)
{
    uint8_t dataRaw[6];

    int8_t result = _theBus->readRegisterRegion(SFE_AS7331_REGISTER_MEAS_MRES1, dataRaw, 6U);

    if (SFE_BUS_OK != result)
        return result;

    if (_mmode == MEAS_MODE_SYND)
    {
        result = readOutConv();

        if (SFE_BUS_OK != result)
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

    return SFE_BUS_OK;
}

int8_t SfeAS7331Driver::readAll(void)
{
    uint8_t dataRaw[8];

    int8_t result = _theBus->readRegisterRegion(SFE_AS7331_REGISTER_MEAS_TEMP, dataRaw, 8U);

    if (SFE_BUS_OK != result)
        return result;

    result = readOutConv();

    if (SFE_BUS_OK != result)
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

    return SFE_BUS_OK;
}

int8_t SfeAS7331Driver::readOutConv(void)
{
    uint8_t tconvRaw[4];

    int8_t result = _theBus->readRegisterRegion(SFE_AS7331_REGISTER_MEAS_OUTCONV_L, tconvRaw, 4U);

    if (SFE_BUS_OK != result)
        return result;

    _outputConversionTime = (uint32_t)(((uint32_t)tconvRaw[3] << 24) | 
                                       ((uint32_t)tconvRaw[2] << 16) |
                                       ((uint32_t)tconvRaw[1] << 8)  | 
                                       tconvRaw[0]);

    return SFE_BUS_OK;
}

as7331_gain_t SfeAS7331Driver::getGain(void)
{
    return _sensorGain;
}

int8_t SfeAS7331Driver::setGain(const as7331_gain_t &gain)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg1_t creg1;

    result = getCReg1(creg1);
    if (SFE_BUS_OK != result)
        return result;

    creg1.gain = gain;

    result = setCReg1(creg1);
    if (SFE_BUS_OK != result)
        return result;

    _sensorGain = gain;

    calculateConversionFactors();

    return SFE_BUS_OK;
}

as7331_conv_clk_freq_t SfeAS7331Driver::getCClk(void)
{
    return _cclk;
}

int8_t SfeAS7331Driver::setCClk(const as7331_conv_clk_freq_t &cclk)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg1(creg3);
    if (SFE_BUS_OK != result)
        return result;

    creg3.cclk = cclk;

    result = setCReg1(creg3);
    if (SFE_BUS_OK != result)
        return result;

    _cclk = cclk;

    calculateConversionFactors();

    return SFE_BUS_OK;
}

as7331_conv_time_t SfeAS7331Driver::getConversionTime(void)
{
    return _conversionTime;
}

int8_t SfeAS7331Driver::setConversionTime(const as7331_conv_time_t &convTime)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg1_t creg1;

    result = getCReg1(creg1);
    if (SFE_BUS_OK != result)
        return result;

    creg1.time = convTime;

    result = setCReg1(creg1);
    if (SFE_BUS_OK != result)
        return result;

    _conversionTime = convTime;

    calculateConversionFactors();

    return SFE_BUS_OK;
}

as7331_ready_pin_mode_t SfeAS7331Driver::getReadyPinMode(void)
{
    return _readyPinMode;
}

int8_t SfeAS7331Driver::setReadyPinMode(const bool &pinMode)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg3(creg3);
    if (SFE_BUS_OK != result)
        return result;

    creg3.rdyod = pinMode;

    result = setCReg3(creg3);
    if (SFE_BUS_OK != result)
        return result;

    _readyPinMode = pinMode;

    return SFE_BUS_OK;
}

bool SfeAS7331Driver::getDigitalDividerEnabled(void)
{
    return _dividerEnabled;
}

// TODO: FIX
int8_t SfeAS7331Driver::setDigitalDividerEnabled(const bool &isEnabled)
{
    // TODO: REDO THIS
    if (_dividerEnabled == isEnabled)
        return SFE_BUS_OK;

    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg2_t creg2;

    result = getCReg2(creg2);
    if (SFE_BUS_OK != result)
        return result;

    creg2.en_div = isEnabled;

    result = setCReg2(creg2);
    if (SFE_BUS_OK != result)
        return result;

    _dividerEnabled = isEnabled;

    if (isEnabled == DIVIDER_ENABLED)
        calculateConversionFactors();

    return SFE_BUS_OK;
}

as7331_divider_val_t SfeAS7331Driver::getDigitalDividerRange(void)
{
    return _dividerRange;
}

// TODO: FIX
int8_t SfeAS7331Driver::setDigitalDividerRange(const as7331_divider_val_t &divider, const bool &setEnableDiv = false)
{
    // TODO: REDO THIS
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg2_t creg2;

    result = getCReg2(&creg2);
    if (SFE_BUS_OK != result)
        return result;

    creg2.div = divider;

    result = setCReg2(&creg2);
    if (SFE_BUS_OK != result)
        return result;

    _config.dividerRange = divider;

    if (setEnableDiv)
        result = setEnableDivider(DIVIDER_ENABLED);
    else
        calculateConversionFactors();

    return SFE_BUS_OK;
}

bool SfeAS7331Driver::getSyndTempConversionEnabled(void)
{
    return _tempConvEnabled;
}

int8_t SfeAS7331Driver::setSyndTempConversionEnabled(const bool &isEnabled)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg2_t creg2;

    result = getCReg2(creg2);
    if (SFE_BUS_OK != result)
        return result;

    creg2.en_tm = isEnabled;

    result = setCReg2(creg2);
    if (SFE_BUS_OK != result)
        return result;

    _tempConvEnabled = isEnabled;

    return SFE_BUS_OK;
}

bool SfeAS7331Driver::getIndexMode(void)
{
    return _indexMode;
}

int8_t SfeAS7331Driver::setIndexMode(const bool &indexMode)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_optreg_t optreg;

    result = getOptIndex(optreg);
    if (SFE_BUS_OK != result)
        return result;

    optreg.init_idx = indexMode;

    result = setOptIndex(optreg);
    if (SFE_BUS_OK != result)
        return result;

    _indexMode = indexMode;

    return SFE_BUS_OK;
}

uint8_t SfeAS7331Driver::getBreakTime(void)
{
    return _breakTime;
}

int8_t SfeAS7331Driver::setBreakTime(const uint8_t &breakTime)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_break_t breakreg;

    result = getBreak(breakreg);
    if (SFE_BUS_OK != result)
        return result;

    breakreg = breakTime;

    result = setBreak(breakreg);
    if (SFE_BUS_OK != result)
        return result;

    _breakTime = breakTime;

    return SFE_BUS_OK;
}

uint8_t SfeAS7331Driver::getNumEdges(void)
{
    return _numEdges;
}

int8_t SfeAS7331Driver::setNumEdges(const uint8_t &numEdges)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_edges_t edgesreg;

    result = getEdges(edgesreg);
    if (SFE_BUS_OK != result)
        return result;

    edgesreg = numEdges;

    result = setEdges(edgesreg);
    if (SFE_BUS_OK != result)
        return result;

    _numEdges = numEdges;

    return SFE_BUS_OK;
}

bool SfeAS7331Driver::getPowerDownState(void)
{
    return _powerDownEnableState;
}

int8_t SfeAS7331Driver::setPowerDownState(const bool &pd)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_osr_t osr;

    result = getOSR(osr);
    if (SFE_BUS_OK != result)
        return result;

    osr.pd = pd;

    result = setOSR(osr);
    if (SFE_BUS_OK != result)
        return result;

    _powerDownEnableState = pd;

    return SFE_BUS_OK;
}

as7331_device_op_state_t SfeAS7331Driver::getOperationMode(void)
{
    return _opMode;
}

int8_t SfeAS7331Driver::setOperationMode(const as7331_device_op_state_t &opMode)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_osr_t osr;

    result = getOSR(osr);
    if (SFE_BUS_OK != result)
        return result;

    osr.dos = opMode;

    result = setOSR(osr);
    if (SFE_BUS_OK != result)
        return result;

    _opMode = opMode;

    return SFE_BUS_OK;
}

as7331_meas_mode_t SfeAS7331Driver::getMeasurementMode(void)
{
    return _mmode;
}

int8_t SfeAS7331Driver::setMeasurementMode(const as7331_meas_mode_t &measMode)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg3(creg3);
    if (SFE_BUS_OK != result)
        return result;

    creg3.mmode = measMode;

    result = setCReg3(creg3);
    if (SFE_BUS_OK != result)
        return result;

    _mmode = measMode;

    return SFE_BUS_OK;
}

bool SfeAS7331Driver::getStandbyState(void)
{
    return _standbyState;
}

int8_t SfeAS7331Driver::setStandbyState(const bool &standby)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_creg3_t creg3;

    result = getCReg3(creg3);
    if (SFE_BUS_OK != result)
        return result;

    creg3.sb = standby;

    result = setCReg3(creg3);
    if (SFE_BUS_OK != result)
        return result;

    _standbyState = standby;

    return SFE_BUS_OK;
}

bool SfeAS7331Driver::getStartState(void)
{
    return _startState;
}

int8_t SfeAS7331Driver::setStartState(const bool &startState)
{
    int8_t result = SFE_BUS_OK;

    sfe_as7331_reg_cfg_osr_t osr;

    result = getOSR(osr);
    if (SFE_BUS_OK != result)
        return result;

    osr.ss = startState;

    result = setOSR(osr);
    if (SFE_BUS_OK != result)
        return result;

    _startState = startState;

    return SFE_BUS_OK;
}

// TODO: add state check
int8_t SfeAS7331Driver::getStatus(sfe_as7331_reg_meas_osr_status_t &statusReg)
{
    int8_t result = SFE_BUS_OK;
    uint8_t statusRaw[2];

    result = readRegisterRegion(SFE_AS7331_REGISTER_MEAS_OSR_STATUS, statusRaw, 2U);

    if (SFE_BUS_OK != result)
        return result;

    // Shift MSB to top of 16-bit register, then OR with LSB.
    statusReg.word = (uint16_t)statusRaw[1] << 8 | statusRaw[0];

    return SFE_BUS_OK;
}

// TODO: add state check
int8_t SfeAS7331Driver::getOSR(sfe_as7331_reg_cfg_osr_t &osrReg)
{
    return _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_OSR, osrReg.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::setOSR(const sfe_as7331_reg_cfg_osr_t &osrReg)
{
    return _theBus->writeRegisterRegion(SFE_AS7331_REGISTER_CFG_OSR, osrReg.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::getCReg1(sfe_as7331_reg_cfg_creg1_t &creg1)
{
    return _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_CREG1, creg1.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::setCReg1(const sfe_as7331_reg_cfg_creg1_t &creg1)
{
    return _theBus->writeRegisterRegion(SFE_AS7331_REGISTER_CFG_CREG1, creg1.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::getCReg2(sfe_as7331_reg_cfg_creg2_t &creg2)
{
    return _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_CREG2, creg2.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::setCReg2(const sfe_as7331_reg_cfg_creg2_t &creg2)
{
    return _theBus->writeRegisterRegion(SFE_AS7331_REGISTER_CFG_CREG2, creg2.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::getCReg3(sfe_as7331_reg_cfg_creg3_t &creg3)
{
    return _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_CREG3, creg3.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::setCReg3(const sfe_as7331_reg_cfg_creg3_t &creg3)
{
    return _theBus->writeRegisterRegion(SFE_AS7331_REGISTER_CFG_CREG3, creg3.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::getBreak(uint8_t &breakReg)
{
    return _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_BREAK, breakReg, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::setBreak(const uint8_t &breakReg)
{
    return _theBus->writeRegisterRegion(SFE_AS7331_REGISTER_CFG_BREAK, breakReg, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::getEdges(uint8_t &edgesReg)
{
    return _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_EDGES, edgesReg, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::setEdges(const uint8_t &edgesReg)
{
    return _theBus->writeRegisterRegion(SFE_AS7331_REGISTER_CFG_EDGES, edgesReg, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::getOptIndex(sfe_as7331_reg_cfg_optreg_t &optReg)
{
    return _theBus->readRegisterRegion(SFE_AS7331_REGISTER_CFG_OPTREG, optReg.byte, 1U);
}

// TODO: add state check
int8_t SfeAS7331Driver::setOptIndex(const sfe_as7331_reg_cfg_optreg_t &optReg)
{
    return _theBus->writeRegisterRegion(SFE_AS7331_REGISTER_CFG_OPTREG, optReg.byte, 1U);
}

int8_t SfeAS7331Driver::readRawUV(const as7331_uv_type &uv_type)
{
    uint8_t uvRawVal[2];
    uint8_t address = 0;
    float *fsr = nullptr;
    float *retUV = nullptr;

    if (_mmode != MEAS_MODE_SYND) // There's a conversion factor already available.
        float *conv = nullptr;

    switch (uv_type)
    {
    case AS7331_UVA:
    default: // Since it's an enum, you can't miscall this function, so default to A.
        address = SFE_AS7331_REGISTER_MEAS_MRES1;
        fsr = &_fsrA;
        retUV = &_uva;
        if (_mmode != MEAS_MODE_SYND)
        {
            conv = &_conversionA;
        }
        break;
    case AS7331_UVB:
        address = SFE_AS7331_REGISTER_MEAS_MRES2;
        fsr = &_fsrB;
        retUV = &_uvb;
        if (_mmode != MEAS_MODE_SYND)
        {
            conv = &_conversionB;
        }
        break;
    case AS7331_UVC:
        address = SFE_AS7331_REGISTER_MEAS_MRES3;
        fsr = &_fsrC;
        retUV = &_uvc;
        if (_mmode != MEAS_MODE_SYND)
        {
            conv = &_conversionC;
        }
        break;
    }

    int8_t result = _theBus->readRegisterRegion(address, uvRawVal, 2U);

    if (SFE_BUS_OK != result)
        return result;

    if (_mmode == MEAS_MODE_SYND)
    {
        result = readOutConv();

        if (SFE_BUS_OK != result)
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
        *retUV = convertRawUVVal((uint16_t)uvRawVal[1] << 8 | uvRawVal[0], conv);
    }

    return SFE_BUS_OK;
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
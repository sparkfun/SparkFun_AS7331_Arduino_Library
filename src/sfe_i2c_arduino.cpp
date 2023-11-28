/*
  SPDX-License-Identifier: MIT

  Copyright (c) 2023 SparkFun Electronics
*/

#include "sfe_i2c_arduino.h"

int8_t SFEBusArduinoI2C::begin(void)
{
    return begin(_i2cBus ? _i2cBus : &Wire );
}

int8_t SFEBusArduinoI2C::begin(TwoWire &wirePort)
{
    return begin(&wirePort);
}

int8_t SFEBusArduinoI2C::begin(TwoWire *i2cBus)
{
    if (!i2cBus)
        return SFE_BUS_E_NULL_PTR;

    _i2cBus = i2cBus;
    
    _i2cBus->begin();

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::end(void)
{
    // Null pointer check.
    if (!_i2cBus)
        return SFE_BUS_E_NULL_PTR;

    _i2cBus->end();

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::ping(const uint8_t devAddr)
{
    // Null pointer check.
    if (_i2cBus == nullptr)
        return SFE_BUS_E_NULL_PTR;
    if (!devAddr)
        return SFE_BUS_E_NULL_PTR;
    // Begin and end transmission to check for ACK response
    _i2cBus->beginTransmission(devAddr);

    return _mapError(_i2cBus->endTransmission());
}

int8_t SFEBusArduinoI2C::ping(const SFEBusDevSettings *devSettings)
{
    // Null pointer check.
    if (!devSettings)
        return SFE_BUS_E_NULL_DEV_SETTINGS;

    SFEBusDevSettingsI2C *pAddr = (SFEBusDevSettingsI2C *)devSettings;

    return ping(pAddr->devAddr);
}

int8_t SFEBusArduinoI2C::writeRegisterBytes(const SFEBusDevSettings *devSettings, const uint8_t regAddr,
                                            const uint8_t *data, const uint32_t numBytes)
{
    // Null pointer check.
    if (!_i2cBus)
        return SFE_BUS_E_NULL_PTR;

    // Null pointer check.
    if (!devSettings)
        return SFE_BUS_E_NULL_DEV_SETTINGS;

    SFEBusDevSettingsI2C *pAddr = (SFEBusDevSettingsI2C *)devSettings;

    uint32_t writeOffset = 0;
    uint32_t bytesToSend = numBytes;
    int8_t result = 0;

    // Start transmission and send register address.
    _i2cBus->beginTransmission(pAddr->devAddr);
    _i2cBus->write(regAddr);

    while (bytesToSend > 0)
    {
        // Limit sendLength to the size of the I2C buffer to send in chunks.
        uint8_t sendLength = (bytesToSend > _i2cBufferSize) ? _i2cBufferSize : bytesToSend;

        // Do the write thing.
        for (uint8_t i = 0; i < sendLength; i++)
            _i2cBus->write(data[writeOffset + i]);

        // If there's still more to send, send a repeat start.
        if (bytesToSend > _i2cBufferSize)
        {
            result = _mapError(_i2cBus->endTransmission(false));
            if (SFE_BUS_OK != result)
                return result;
        }

        writeOffset += sendLength;
        bytesToSend -= sendLength;
    }

    return _mapError(_i2cBus->endTransmission());
}

int8_t SFEBusArduinoI2C::readRegisterBytes(const SFEBusDevSettings *devSettings, const uint8_t regAddr, uint8_t *data,
                                           const uint32_t numBytes)
{
    // Null pointer check.
    if (!_i2cBus)
        return SFE_BUS_E_NULL_PTR;

    // Null pointer check.
    if (!devSettings)
        return SFE_BUS_E_NULL_DEV_SETTINGS;

    // Null pointer check.
    if (!data)
        return SFE_BUS_E_NULL_DATA_BUFFER;

    SFEBusDevSettingsI2C *pAddr = (SFEBusDevSettingsI2C *)devSettings;

    // Start transmission and send register address.
    _i2cBus->beginTransmission(pAddr->devAddr);
    _i2cBus->write(regAddr);

    // Repeat start condition, return if there's an error.
    int8_t result = _mapError(_i2cBus->endTransmission(false));
    if (SFE_BUS_OK != result)
        return result;

    uint32_t bytesLeftToRead = numBytes;
    uint32_t readOffset = 0;

    while (bytesLeftToRead > 0)
    {
        // Limit readLength to the size of the I2C buffer to read in chunks.
        uint8_t readLength = (bytesLeftToRead > _i2cBufferSize) ? _i2cBufferSize : bytesLeftToRead;

        // Request bytes, then read them into the data buffer.
        uint32_t numRead = _i2cBus->requestFrom(pAddr->devAddr, readLength);

        if (numRead < readLength)
            return SFE_BUS_W_UNDER_READ;

        if (_i2cBus->available())
        {
            for (uint8_t i = 0; i < readLength; i++)
                data[readOffset + i] = _i2cBus->read();
        }
        else
            return SFE_BUS_E_NO_RESPONSE;

        readOffset += readLength;
        bytesLeftToRead -= readLength;
    }

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::writeBytes(const SFEBusDevSettings *devSettings, const uint8_t *data, uint32_t numBytes)
{
    // Null pointer check.
    if (!_i2cBus)
        return SFE_BUS_E_NULL_PTR;

    // Null pointer check.
    if (!devSettings)
        return SFE_BUS_E_NULL_DEV_SETTINGS;

    SFEBusDevSettingsI2C *pAddr = (SFEBusDevSettingsI2C *)devSettings;

    uint32_t writeOffset = 0;
    uint32_t bytesToSend = numBytes;
    int8_t result = 0;

    // Start transmission.
    _i2cBus->beginTransmission(pAddr->devAddr);

    while (bytesToSend > 0)
    {
        // Limit sendLength to the size of the I2C buffer to send in chunks.
        uint8_t sendLength = (bytesToSend > _i2cBufferSize) ? _i2cBufferSize : bytesToSend;

        // Do the write thing.
        for (uint8_t i = 0; i < sendLength; i++)
            _i2cBus->write(data[writeOffset + i]);

        // If there's still more to send, send a repeat start.
        if (bytesToSend > _i2cBufferSize)
        {
            result = _mapError(_i2cBus->endTransmission(false));
            if (SFE_BUS_OK != result)
                return result;
        }

        writeOffset += sendLength;
        bytesToSend -= sendLength;
    }

    return _mapError(_i2cBus->endTransmission());
}

int8_t SFEBusArduinoI2C::readBytes(const SFEBusDevSettings *devSettings, uint8_t *data, uint32_t numBytes)
{
    // Null pointer check.
    if (!_i2cBus)
        return SFE_BUS_E_NULL_PTR;

    // Null pointer check.
    if (!devSettings)
        return SFE_BUS_E_NULL_DEV_SETTINGS;

    SFEBusDevSettingsI2C *pAddr = (SFEBusDevSettingsI2C *)devSettings;

    // Start transmission.
    _i2cBus->beginTransmission(pAddr->devAddr);

    // Repeat start condition, return if there's an error.
    int8_t result = _mapError(_i2cBus->endTransmission(false));
    if (SFE_BUS_OK != result)
        return result;

    uint32_t bytesLeftToRead = numBytes;
    uint32_t readOffset = 0;

    while (bytesLeftToRead > 0)
    {
        // Limit readLength to the size of the I2C buffer to read in chunks.
        uint8_t readLength = (bytesLeftToRead > _i2cBufferSize) ? _i2cBufferSize : bytesLeftToRead;

        // Request bytes, then read them into the data buffer.
        uint32_t numRead = _i2cBus->requestFrom(pAddr->devAddr, readLength);

        if (numRead < readLength)
            return SFE_BUS_W_UNDER_READ;

        if (_i2cBus->available())
        {
            for (uint8_t i = 0; i < readLength; i++)
                data[readOffset + i] = _i2cBus->read();
        }
        else
            return SFE_BUS_E_NO_RESPONSE;

        readOffset += readLength;
        bytesLeftToRead -= readLength;
    }

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::setBufferSize(const uint32_t bufferSize)
{
    _i2cBufferSize = bufferSize;

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::getBufferSize(uint32_t *bufferSize)
{
    *bufferSize = _i2cBufferSize;

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::setBusFrequency(const uint32_t frequency)
{
    if (!_i2cBus)
        return SFE_BUS_E_NULL_PTR;

    _i2cBus->setClock(frequency);
    _busFrequency = frequency;

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::getBusFrequency(uint32_t *frequency)
{
    *frequency = _busFrequency;

    return SFE_BUS_OK;
}

int8_t SFEBusArduinoI2C::_mapError(const uint8_t error)
{
    if (!error)
        return SFE_BUS_OK;
    else if (error == 1)
        return SFE_BUS_E_DATA_TOO_LONG;
    else if ((error == 2) || (error == 3))
        return SFE_BUS_E_NO_RESPONSE;
    else if (error == 5)
        return SFE_BUS_E_TIMEOUT;
    else
        return SFE_BUS_E_UNKNOWN;
}

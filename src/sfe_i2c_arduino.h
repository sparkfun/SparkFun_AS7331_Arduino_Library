/*
  SPDX-License-Identifier: MIT

  Copyright (c) 2023 SparkFun Electronics
*/

#pragma once

#include <Arduino.h>
#include "sfe_i2c.h"
#include <Wire.h>

/// @brief An I2C communication bus implementation for Arduino
class SFEBusArduinoI2C : public SFEBusI2C
{
  public:
    /// @brief Empty Constructor.
    SFEBusArduinoI2C(void) : _i2cBus{nullptr}{};

    /// @brief Passing in a TwoWire Object.
    SFEBusArduinoI2C(TwoWire &wirePort) : _i2cBus{&wirePort}{};

    /// @brief Begin the I2C object with the default Wire object.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t begin(void);

    /// @brief Begin the I2C object with the inputted Wire object.
    /// @param wirePort I2C object to use for this bus.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t begin(TwoWire &wirePort);

    /// @brief Begin the I2C object with the inputted Wire object pointer.
    /// @param  i2cBus I2C bus pointer.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t begin(TwoWire *i2cBus);
    
    /// @brief End the I2C object.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t end(void);

    /// @brief Pings I2C device and looks for an ACK response.
    /// @param devAddr Address to ping.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t ping(const uint8_t devAddr);

    /// @brief Pings I2C device and looks for an ACK response.
    /// @param devSettings Settings object containing the address to ping.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t ping(const SFEBusDevSettings *devSettings);

    /// @brief Writes a number of bytes starting at the given register address.
    /// @param devSettings I2C Settings object containing the address of the device.
    /// @param regAddr The register address to write to.
    /// @param data Data buffer to write to registers.
    /// @param numBytes Number of bytes to write.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t writeRegisterBytes(const SFEBusDevSettings *devSettings, const uint8_t regAddr, const uint8_t *data, const uint32_t numBytes);

    /// @brief Reads a number of bytes starting at the given register address.
    /// @param devSettings I2C Settings object containing the address of the device.
    /// @param regAddr The first register address to read from.
    /// @param data Data buffer to read from registers.
    /// @param numBytes Number of bytes to read.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t readRegisterBytes(const SFEBusDevSettings *devSettings, const uint8_t regAddr, uint8_t *data, const uint32_t numBytes);

    /// @brief Writes a number of bytes to a device that doesn't use registers for communications.
    /// @param devSettings I2C Settings object containing the address of the device.
    /// @param data Data buffer to write to registers.
    /// @param numBytes Number of bytes to write.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t writeBytes(const SFEBusDevSettings *devSettings, const uint8_t *data, const uint32_t numBytes);

    /// @brief Reads a number of bytes to a device that doesn't use registers for communications.
    /// @param devSettings I2C Settings object containing the address of the device.
    /// @param data Data buffer to read from registers.
    /// @param numBytes Number of bytes to read.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t readBytes(const SFEBusDevSettings *devSettings, uint8_t *data, const uint32_t numBytes);

    /// @brief Changes the I2C buffer size.
    /// @param bufferSize New buffer size.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t setBufferSize(const uint32_t bufferSize);

    /// @brief Returns the I2C buffer size.
    /// @param bufferSize Buffer to return the buffer... yep.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t getBufferSize(uint32_t *bufferSize);

    /// @brief Changes the Bus transmit frequency.
    /// @param frequency New bus frequency.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t setBusFrequency(const uint32_t frequency);

    /// @brief Returns the Bus transmit frequency.
    /// @param frequency Buffer to return the frequency.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t getBusFrequency(uint32_t *frequency);

  private:
    TwoWire* _i2cBus;

    /// @brief Maps the TwoWire interface error scheme to the common bus error scheme.
    /// @param error TwoWire error code.
    /// @return 0 for success, negative for failure, positive for warning.
    int8_t _mapError(const uint8_t error);
};

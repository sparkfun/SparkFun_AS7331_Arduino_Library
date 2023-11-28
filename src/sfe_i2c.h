/*
  SPDX-License-Identifier: MIT

  Copyright (c) 2023 SparkFun Electronics
*/

#pragma once

#include "sfe_bus.h"

#define SFE_BUS_I2C_DEFAULT_ADDRESS 0x00
#define SFE_BUS_I2C_DEFAULT_I2C_SPEED 100000
#define SFE_BUS_I2C_BUFFER_SIZE 32

// @brief A simple bus address implementation for a generic I2C.
class SFEBusDevSettingsI2C : public SFEBusDevSettings
{
  public:
    uint8_t devAddr = SFE_BUS_I2C_DEFAULT_ADDRESS; // Default I2C Address
    uint32_t maxDataRate = SFE_BUS_I2C_DEFAULT_I2C_SPEED; // Default I2C Speed
};

/// @brief An abstract interface for an I2C communication bus
class SFEBusI2C : public SFEBus
{
  public:
    /// @brief Pings I2C device and looks for an ACK response.
    /// @param devAddr Address to ping.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t ping(const uint8_t devAddr) = 0;

    /// @brief Pings I2C device and looks for an ACK response.
    /// @param devSettings Settings of device to ping.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t ping(const SFEBusDevSettings *devSettings) = 0;

    /// @brief Changes the I2C buffer size.
    /// @param bufferSize New buffer size.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t setBufferSize(const uint32_t bufferSize) = 0;

    /// @brief Returns the I2C buffer size.
    /// @param bufferSize Buffer to return the size of the transmit buffer.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t getBufferSize(uint32_t *bufferSize) = 0;

    /// @brief Changes the Bus transmit frequency.
    /// @param frequency New bus frequency.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t setBusFrequency(const uint32_t frequency) = 0;

    /// @brief Returns the Bus transmit frequency.
    /// @param frequency Buffer to return the frequency.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t getBusFrequency(uint32_t *frequency) = 0;

  protected:
    uint32_t _i2cBufferSize = SFE_BUS_I2C_BUFFER_SIZE; // Default Buffer Size
    uint32_t _busFrequency = SFE_BUS_I2C_DEFAULT_I2C_SPEED; // Default Speed is 100kHz
};

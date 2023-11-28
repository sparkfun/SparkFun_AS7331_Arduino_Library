/*
  SPDX-License-Identifier: MIT

  Copyright (c) 2023 SparkFun Electronics

  A pure virtual base class for implementing a common communication interface
  in SparkFun products.
*/

#pragma once

#include <stdint.h>

// Error and warning codes
#define SFE_BUS_OK 0
#define SFE_BUS_E_UNKNOWN -1
#define SFE_BUS_E_NULL_PTR -2
#define SFE_BUS_E_TIMEOUT -3
#define SFE_BUS_E_NO_RESPONSE -4
#define SFE_BUS_E_DATA_TOO_LONG -5
#define SFE_BUS_E_NULL_DEV_SETTINGS -6
#define SFE_BUS_E_NULL_DATA_BUFFER -7
#define SFE_BUS_W_UNKNOWN 1
#define SFE_BUS_W_UNDER_READ 2
#define SFE_BUS_W_NOT_ENABLED 3

/// @brief An abstract Bus address class for enabling multiple types of addresses.
class SFEBusDevSettings{}; // Nothing to see here...

/// @brief An abstract interface for a communication bus
class SFEBus
{
  public:
    /// @brief Begin bus.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t begin(void) = 0;

    /// @brief End bus.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t end(void) = 0;

    /// @brief Writes a number of bytes starting at the given register address.
    /// @param devSettings Settings of the device.
    /// @param regAddr The first register address to write to.
    /// @param data Data buffer to write to registers.
    /// @param numBytes Number of bytes to write.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t writeRegisterBytes(const SFEBusDevSettings *devSettings, const uint8_t regAddr, const uint8_t *data, const uint32_t numBytes) = 0;

    /// @brief Reads a number of bytes starting at the given register address.
    /// @param devSettings Settings of the device.
    /// @param regAddr The first register address to read from.
    /// @param data Data buffer to read from registers.
    /// @param numBytes Number of bytes to read.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t readRegisterBytes(const SFEBusDevSettings *devSettings, const uint8_t regAddr, uint8_t *data, const uint32_t numBytes) = 0;

    /// @brief Writes a number of bytes to a device that doesn't use registers for communications.
    /// @param devSettings Settings of the device.
    /// @param data Data buffer to write from registers.
    /// @param numBytes Number of bytes to write.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t writeBytes(const SFEBusDevSettings *devSettings, const uint8_t *data, const uint32_t numBytes) = 0;

    /// @brief Reads a number of bytes to a device that doesn't use registers for communications.
    /// @param devSettings Settings of the device.
    /// @param data Data buffer to read from registers.
    /// @param numBytes Number of bytes to read.
    /// @return 0 for success, negative for failure, positive for warning.
    virtual int8_t readBytes(const SFEBusDevSettings *devSettings, uint8_t *data, const uint32_t numBytes) = 0;
};

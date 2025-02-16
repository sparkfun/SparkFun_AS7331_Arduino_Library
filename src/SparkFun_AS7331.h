/*/**
 * @file SparkFun_AS7331.h
 * @brief Arduino-specific implementation for the SparkFun AS7331 UV Sensor.
 *
 * @details
 * This file provides the Arduino-specific implementation of the AS7331 driver class.
 * The SfeAS7331ArdI2C class inherits from SfeAS7331Driver and implements the I2C
 * communication interface using Arduino's Wire library.
 *
 * Key features:
 * - Arduino I2C initialization
 * - Device address configuration
 * - Connection verification
 * - Toolkit integration
 *
 * @section Class SfeAS7331ArdI2C Class
 * - begin(): Initializes I2C communication
 * - isConnected(): Verifies sensor connection
 * - setDeviceAddress(): Configures I2C address
 * - getDeviceAddress(): Returns current I2C address
 *
 * @section Dependencies Dependencies
 * - Arduino.h
 * - SparkFun_Toolkit.h
 * - sfeAS7331.h
 *
 * @author SparkFun Electronics
 * @date 2023
 * @copyright Copyright (c) 2023-2025, SparkFun Electronics Inc. All rights reserved.
 *
 * @section License License
 * SPDX-License-Identifier: MIT
 *
 * @section Product_Links Product Links
 * - Qwiic 1x1: https://www.sparkfun.com/products/23517
 * - Qwiic Mini: https://www.sparkfun.com/products/23518
 *
 * @see https://github.com/sparkfun/SparkFun_AS7331_Arduino_Library
 */

#pragma once

// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevAS7331.h"
#include <Arduino.h>
// clang-format on

/**
 * @class SfeAS7331ArdI2C
 * @brief Arduino I2C implementation for the AS7331 UV sensor.
 *
 * @details
 * This class provides Arduino-specific I2C communication implementation for the AS7331 UV sensor.
 * It inherits from the base driver class and implements the I2C interface using Arduino's Wire library.
 * The class manages device addressing and connection verification.
 *
 * Example usage:
 * @code
 * SfeAS7331ArdI2C uvSensor;
 * if (uvSensor.begin()) {
 *     // Sensor initialized successfully
 * }
 * @endcode
 *
 * @note This class uses the Arduino Wire library for I2C communication
 *
 * @see sfDevAS7331
 * @see TwoWire
 *
 */
class SfeAS7331ArdI2C : public sfDevAS7331
{
  public:
    SfeAS7331ArdI2C()
    {
    }

    /**
     * @brief Initializes the AS7331 UV sensor with I2C communication.
     *
     * @details
     * This method performs the following initialization steps:
     * 1. Initializes I2C communication with specified address and Wire port
     * 2. Enables repeat start support
     * 3. Sets up communication bus
     * 4. Verifies device connection
     * 5. Calls base class initialization
     *
     * @param address I2C address of the device (default: kDefaultAS7331Addr)
     * @param wirePort TwoWire instance to use for I2C communication (default: Wire)
     *
     * @return true If initialization successful
     * @return false If any initialization step fails
     *
     * Example:
     * @code
     * SfeAS7331ArdI2C sensor;
     * if (!sensor.begin()) {
     *     Serial.println("Sensor initialization failed!");
     *     while (1); // halt
     * }
     * @endcode
     */
    bool begin(const uint8_t &address = kDefaultAS7331Addr, TwoWire &wirePort = Wire)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        // Device supports repeat starts, enable it.
        _theI2CBus.setStop(false);

        setCommunicationBus(&_theI2CBus);

        if (!isConnected())
            return false;

        return sfDevAS7331::begin();
    }

    /**
     * @brief Checks if the AS7331 UV sensor is connected and responding.
     *
     * @details
     * This method performs two checks:
     * 1. Attempts to ping the device at the current I2C address
     * 2. Verifies the device ID matches the expected AS7331 ID
     *
     * @return true If device responds to ping and returns correct device ID
     * @return false If communication fails or device ID is incorrect
     *
     * Example:
     * @code
     * SfeAS7331ArdI2C sensor;
     * if (!sensor.isConnected()) {
     *     Serial.println("Device not found or incorrect ID!");
     *     return;
     * }
     * @endcode
     */
    bool isConnected(void)
    {
        if (_theI2CBus.ping() != ksfTkErrOk)
            return false;

        return (kDefaultAS7331DeviceID == getDeviceID());
    }

    /**
     * @brief Sets the I2C address for communicating with the AS7331 sensor.
     *
     * @details
     * This method validates and sets the I2C address for the sensor. Only specific addresses are allowed:
     * - kDefaultAS7331Addr (default address)
     * - kSecondaryAS7331Addr
     * - kTertiaryAS7331Addr
     * - kQuaternaryAS7331Addr
     *
     * @param deviceAddress The I2C address to set
     * @return true If address is valid and was set successfully
     * @return false If address is invalid
     *
     * Example:
     * @code
     * SfeAS7331ArdI2C sensor;
     * // Change to secondary address
     * if (!sensor.setDeviceAddress(kSecondaryAS7331Addr)) {
     *     Serial.println("Invalid address!");
     *     return;
     * }
     * @endcode
     */
    bool setDeviceAddress(const uint8_t &deviceAddress)
    {
        switch (deviceAddress)
        {
        // If it's any of the allowed addresses, set it.
        case kDefaultAS7331Addr:
        case kSecondaryAS7331Addr:
        case kTertiaryAS7331Addr:
        case kQuaternaryAS7331Addr:
            _theI2CBus.setAddress(deviceAddress);
            break;
        default: // If it's invalid, return false.
            return false;
            break;
        }
        return true;
    }

    /**
     * @brief Gets the currently configured I2C address of the AS7331 sensor.
     *
     * @details
     * Returns the I2C address currently being used to communicate with the sensor.
     * The address will be one of:
     * - kDefaultAS7331Addr
     * - kSecondaryAS7331Addr
     * - kTertiaryAS7331Addr
     * - kQuaternaryAS7331Addr
     *
     * @return uint8_t The current I2C address
     *
     * Example:
     * @code
     * SfeAS7331ArdI2C sensor;
     * uint8_t address = sensor.getDeviceAddress();
     * Serial.print("Current I2C address: 0x");
     * Serial.println(address, HEX);
     * @endcode
     */
    uint8_t getDeviceAddress(void)
    {
        return _theI2CBus.address();
    }

  private:
    /**
     * @brief Arduino I2C bus interface instance for the AS7331 sensor.
     *
     * @details
     * This member handles the low-level I2C communication between the Arduino and the AS7331 sensor.
     *
     * The bus interface is configured during begin() and used by all communication methods.
     *
     * @see sfTkArdI2C
     * @see begin()
     */
    sfTkArdI2C _theI2CBus;
};

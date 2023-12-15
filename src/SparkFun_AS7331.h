/*
    SparkFun Spectral UV Sensor - AS7331

    Qwiic 1x1
    https://www.sparkfun.com/products/23517
    Qwiic Mini
    https://www.sparkfun.com/products/23518

    Repository
    https://github.com/sparkfun/SparkFun_AS7331_Arduino_Library

    SPDX-License-Identifier: MIT

    Copyright (c) 2023 SparkFun Electronics

    Name: SparkFun_AS7331.h

    Description:
    The SfeAS7331ArdI2C class defines the Arduino specific behavior for 
    initializing and interadcting with devices.

*/

#pragma once

#include "sfeAS7331.h"
#include <Arduino.h>
#include <SparkFun_Toolkit.h>

class SfeAS7331ArdI2C : public SfeAS7331Driver
{
  public:
    SfeAS7331ArdI2C()
    {
    }

    /// @brief  Sets up Arduino I2C driver using the specified I2C address then calls the super class begin.
    /// @param address Address of the I2C device.
    /// @return True if successful, false otherwise.
    bool begin(const uint8_t &address = kDefaultAS7331Addr, TwoWire &wirePort = Wire)
    {
        if (_theI2CBus.init(wirePort, address) != kSTkErrOk)
            return false;

        // Device supports repeat starts, enable it.
        _theI2CBus.setStop(false);

        setCommunicationBus(&_theI2CBus);

        isConnected();

        return SfeAS7331Driver::begin(address);
    }

    /// @brief Checks to see if the AS7331 is connected.
    /// @return True if successful, false otherwise.
    bool isConnected(void)
    {
        if (_theI2CBus.ping() != kSTkErrOk)
            return false;

        return (kDefaultAS7331DeviceID == getDeviceID());
    }

  private:
    sfeTkArdI2C _theI2CBus;
};

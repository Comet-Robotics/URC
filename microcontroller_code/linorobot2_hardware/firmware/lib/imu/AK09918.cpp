/*
    AK09918.cpp
    A library for Grove - IMU 9DOF(ICM20600 + AK09918)

    Copyright (c) 2018 seeed technology inc.
    Website    : www.seeed.cc
    Author     : Jerry Yip
    Create Time: 2018-06
    Version    : 0.1
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/


#include "AK09918.h"

AK09918::AK09918() {
    _addr = AK09918_I2C_ADDR;
}

AK09918_err_type_t AK09918::initialize(AK09918_mode_type_t mode) {
    if (!I2Cdev::writeByte(_addr, AK09918_CNTL2, mode)) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

bool AK09918::testConnection() {
  if (I2Cdev::readByte(_addr, AK09918_WIA1, _buffer) == 1) {
        return (_buffer[0] == 0x48);
    }
    return false;
}

void AK09918::getHeading(int16_t* x, int16_t* y, int16_t* z) {
    I2Cdev::readBytes(_addr, AK09918_HXL, 8, _buffer);
    *x = (int16_t)(_buffer[1] << 8 | _buffer[0]);
    *y = (int16_t)(_buffer[3] << 8 | _buffer[2]);
    *z = (int16_t)(_buffer[5] << 8 | _buffer[4]);
}

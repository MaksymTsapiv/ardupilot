/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "AP_InertialSensor.h"
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_InertialSensor_LSM6DS33.h"

#include <inttypes.h>
#include <utility>



const extern AP_HAL::HAL& hal;


///
/// Accelerometr Gyroscope register definition
#define INT1_CTRL         0x0D
#define INT2_CTRL         0x0E
#define WHO_AM_I          0x0F
#define CTRL1_XL          0x10
#define CTRL2_G           0x11
#define CTRL3_C           0x12
#define CTRL4_C           0x13
#define CTRL5_C           0x14
#define CTRL6_C           0x15
#define CTRL7_G           0x16
#define CTRL8_XL          0x17
#define CTRL9_XL          0x18
#define CTRL10_C          0x19

#define STATUS_REG        0x1E

#define OUT_TEMP_L        0x20
#define OUT_TEMP_H        0x21
#define OUTX_L_G          0x22
#define OUTX_H_G          0x23
#define OUTY_L_G          0x24
#define OUTY_H_G          0x25
#define OUTZ_L_G          0x26
#define OUTZ_H_G          0x27
#define OUTX_L_XL         0x28
#define OUTX_H_XL         0x29
#define OUTY_L_XL         0x2A
#define OUTY_H_XL         0x2B
#define OUTZ_L_XL         0x2C
#define OUTZ_H_XL         0x2D

#define FIFO_STATUS1      0x3A
#define FIFO_STATUS2      0x3B


AP_InertialSensor_LSM6DS33::AP_InertialSensor_LSM6DS33(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))

{
}


AP_InertialSensor_LSM6DS33::~AP_InertialSensor_LSM6DS33()
{
}



AP_InertialSensor_Backend *AP_InertialSensor_LSM6DS33::probe(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)

{
    // AP_HAL::panic("LSM6D33 dummy sensor");

    if (!dev){
        return nullptr;
    }

    AP_InertialSensor_LSM6DS33 *sensor
        = new AP_InertialSensor_LSM6DS33(imu, std::move(dev));
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
    
}

bool AP_InertialSensor_LSM6DS33::_accel_gyro_init()
{
    // AP_HAL::panic("LSM6D33 dummy sensor");

    //// LSM6DS33/LSM6DSO gyro

    // ODR = 1000 (1.66 kHz (high performance))
    // FS_G = 11 (2000 dps)
    _dev->write_register(CTRL2_G, 0b10001100);
    hal.scheduler->delay(5);

    // defaults
    _dev->write_register(CTRL7_G, 0b00000000);
    hal.scheduler->delay(5);

    //// LSM6DS33/LSM6DSO accelerometer

    // ODR = 1000 (1.66 kHz (high performance))
    // FS_XL = 11 (8 g full scale)
    _dev->write_register(CTRL1_XL, 0b10001100);
    hal.scheduler->delay(5);
    
    //// common

    // IF_INC = 1 (automatically increment address register)
    _dev->write_register(CTRL3_C, 0b00000100);
    hal.scheduler->delay(5);

    return true;
}

bool AP_InertialSensor_LSM6DS33::_init_sensor(void)
{
    
    _accel_gyro_init();

    return true;
}


/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_LSM6DS33::update(void)
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}


/*
  startup the sensor
 */
void AP_InertialSensor_LSM6DS33::start(void)
{

    if (!_imu.register_gyro(_gyro_instance, 800, _dev->get_bus_id_devtype(DEVTYPE_GYR_LSM6DS33)) ||
        !_imu.register_accel(_accel_instance, 800, _dev->get_bus_id_devtype(DEVTYPE_ACC_LSM6DS33))) {
        return;
    }

    // start the timer process to read samples
    _dev->register_periodic_callback(1250, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DS33::_accumulate_accel, void));

    AP_HAL::panic("LSM6D33 dummy sensor");

}

// Accumulate values from accels and gyros
void AP_InertialSensor_LSM6DS33::_accumulate_gyro (void)
{

}

void AP_InertialSensor_LSM6DS33::_accumulate_accel (void)
{
    
}

#endif

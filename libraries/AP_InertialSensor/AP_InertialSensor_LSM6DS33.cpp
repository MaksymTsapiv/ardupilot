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
#define WHO_AM_I_VALUE    0x69
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
#define STATUS_REG_G_DRD  (0x1 << 1)
#define STATUS_REG_A_DRD  (0x1)

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



AP_InertialSensor_LSM6DS33::AP_InertialSensor_LSM6DS33(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(std::move(rotation))

{
}


AP_InertialSensor_LSM6DS33::~AP_InertialSensor_LSM6DS33()
{
}



AP_InertialSensor_Backend *AP_InertialSensor_LSM6DS33::probe(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, enum Rotation rotation)

{
    // AP_HAL::panic("LSM6D33 dummy sensor");

    if (!dev){
        return nullptr;
    }

    AP_InertialSensor_LSM6DS33 *sensor
        = new AP_InertialSensor_LSM6DS33(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
    
}

void AP_InertialSensor_LSM6DS33::_set_accel_scale(accel_scale scale)
{
    switch (scale){
        case A_SCALE_2G:
            _accel_scale = 0.061;
            break;
        case A_SCALE_4G:
            _accel_scale = 0.122;
            break;
        case A_SCALE_8G:
            _accel_scale = 0.244;
            break;
        case A_SCALE_16G:
            _accel_scale = 0.488;
            break;

    }

    /* convert to mG/LSB to g/LSB */
    _accel_scale /= 1000;

    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale *= GRAVITY_MSS;
}

void AP_InertialSensor_LSM6DS33::_set_gyro_scale(gyro_scale scale)
{
    /* scales values from datasheet in mdps/digit */
    switch (scale) {
        case G_SCALE_245DPS:
            _gyro_scale = 8.75;
            break;
        case G_SCALE_500DPS:
            _gyro_scale = 17.50;
            break;
        case G_SCALE_2000DPS:
            _gyro_scale = 70;
            break;
    }

    /* convert mdps/digit to dps/digit */
    _gyro_scale /= 1000;
    /* convert dps/digit to (rad/s)/digit */
    _gyro_scale *= DEG_TO_RAD;
}

bool AP_InertialSensor_LSM6DS33::_accel_gyro_init()
{
    uint8_t data = 0;
    _dev->read_registers(WHO_AM_I, &data, 1);
    if (data != WHO_AM_I_VALUE) {
        AP_HAL::panic("AP_InertialSensor_LSM6DS33: could not find LSM6DS33 sensor");
    }

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

    _set_gyro_scale(G_SCALE_2000DPS);
    _set_accel_scale(A_SCALE_8G);

    return true;
}

bool AP_InertialSensor_LSM6DS33::_init_sensor(void)
{
    
    _accel_gyro_init();

    return true;
}

bool AP_InertialSensor_LSM6DS33::_accel_data_ready()
{
    uint8_t status = 0;
    _dev->read_registers(STATUS_REG, &status, 1);

    hal.console->printf("accel status\n");
    hal.console->printf("%02x", status & STATUS_REG_A_DRD);
    hal.console->printf("\n");

    return status & STATUS_REG_A_DRD;
}

bool AP_InertialSensor_LSM6DS33::_gyro_data_ready()
{
    uint8_t status = 0;
    _dev->read_registers(STATUS_REG, &status, 1);

    hal.console->printf("gyro status\n");
    hal.console->printf("%02x", status & STATUS_REG_G_DRD);
    hal.console->printf("\n");

    return status & STATUS_REG_G_DRD;
}


/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_LSM6DS33::update(void)
{   if (_gyro_data_ready()) {
        update_gyro(_gyro_instance);
    }
    if (_accel_data_ready()) {
        update_accel(_accel_instance);
    }

    return true;
}


/*
  startup the sensor
 */
void AP_InertialSensor_LSM6DS33::start(void)
{

    if (!_imu.register_gyro(_gyro_instance, 1660, _dev->get_bus_id_devtype(DEVTYPE_GYR_LSM6DS33)) ||
        !_imu.register_accel(_accel_instance, 1660, _dev->get_bus_id_devtype(DEVTYPE_ACC_LSM6DS33))) {
        return;
    }
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    // start the timer process to read samples
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DS33::_accumulate_accel, void));
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DS33::_accumulate_gyro, void));

//    AP_HAL::panic("LSM6D33 dummy sensor");

}


void AP_InertialSensor_LSM6DS33::_accumulate_accel (void)
{
    struct sensor_raw_data raw_data = { };
    const uint8_t reg = OUTX_L_XL;

    if (!_dev->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data))) {
        return;
    }

    hal.console->printf("accel data \n");

    hal.console->printf("%02x:%02x:%02x ", raw_data.x, raw_data.y, -raw_data.z);
    hal.console->printf("\n");

    Vector3f accel_data(raw_data.x, raw_data.y, -raw_data.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data, AP_HAL::micros64());

}

void AP_InertialSensor_LSM6DS33::_accumulate_gyro (void)
{

    struct sensor_raw_data raw_data = { };
    const uint8_t reg = OUTX_L_G;

    if (!_dev->transfer(&reg, 1, (uint8_t *) &raw_data, sizeof(raw_data))) {
        return;
    }

    hal.console->printf("gyro data\n");
    hal.console->printf("%02x:%02x:%02x ", raw_data.x, raw_data.y, -raw_data.z);
    hal.console->printf("\n");

    Vector3f gyro_data(raw_data.x, raw_data.y, -raw_data.z);
    gyro_data *= _gyro_scale;

    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data, AP_HAL::micros64());

}

#endif

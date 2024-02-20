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
#define FIFO_CTRL3        0x08
#define FIFO_CTRL5        0x0A
#define FIFO_DATA_OUT_L   0x3E
#define FIFO_DATA_OUT_H   0x3F


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

    if (!_imu.register_gyro(_gyro_instance, 1660, _dev->get_bus_id_devtype(DEVTYPE_GYR_LSM6DS33)) ||
        !_imu.register_accel(_accel_instance, 1660, _dev->get_bus_id_devtype(DEVTYPE_ACC_LSM6DS33))) {
        return;
    }
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    // start the timer process to read samples
    _configure_fifo();
    _dev->register_periodic_callback(5000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DS33::_read_fifo, void));

//    AP_HAL::panic("LSM6D33 dummy sensor");

}



bool AP_InertialSensor_LSM6DS33::_configure_fifo()
{

    // When Bypass mode is enabled, the FIFO is not used, the buffer content is cleared, and it
    // remains empty until another mode is selected.
    // Bypass mode is selected when the FIFO_MODE_[2:0] bits are set to 000b. When this mode
    // is enabled, the FIFO_STATUS2 register contains the value 10h (FIFO empty).
    // Bypass mode must be used in order to stop and reset the FIFO buffer when a different
    // mode is operating. Note that placing the FIFO buffer in Bypass mode, the whole buffer
    // content is cleared.

    bool status = _dev->write_register(FIFO_CTRL5, 0b00000000);
    if (!status) {
        DEV_PRINTF("LSM6DS33: Unable to clear FIFO (FIFO_CTRL5)\n");
        return false;
    }

    // 2. Choose the FIFO ODR through the ODR_FIFO_[3:0] bits in the FIFO_CTRL5 register. It’s
    // recommended to set the ODR_FIFO_[3:0] bits of the FIFO_CTRL5 register to 1000b in order to
    // set the FIFO trigger ODR to 1600 Hz if the internal trigger (accelerometer/gyroscope
    // data-ready) is used and Gyroscope ODR = 1.66 kHz, Accelerometer ODR = 1.66 kHz;
    // 3. Set the FIFO_MODE_[2:0] bits in the FIFO_CTRL5 register to 001b to enable the FIFO
    // mode.
    status = _dev->write_register(FIFO_CTRL5, 0b01000001);
    if (!status) {
        DEV_PRINTF("LSM6DS33: Unable to configure FIFO mode or ODR (FIFO_CTRL5)\n");
        return false;
    }

    // 1. Choose the decimation factor for each sensor through the decimation bits in the
    // FIFO_CTRL3 and FIFO_CTRL4 registers. For no decimation, both the DEC_FIFO_GYRO[2:0] and the
    // DEC_FIFO_XL[2:0] fields of the FIFO_CTRL3 register have to be set to 001b;
    status = _dev->write_register(FIFO_CTRL3, 0b00001001);
    if (!status) {
        DEV_PRINTF("LSM6DS33: Unable to configure FIFO decimation factor (FIFO_CTRL3)\n");
        return false;
    }



    // To guarantee the correct acquisition of data during the switching into and out of FIFO mode,
    // the first set of data acquired must be discarded.
    struct sensor_raw_data raw_data[2] = {0};
    status = _dev->read_registers(FIFO_DATA_OUT_L, (uint8_t *) &raw_data, sizeof(raw_data));
    if (!status) {
        DEV_PRINTF("LSM6DS33: Unable to read the first set of FIFO data (FIFO_DATA_OUT_L, FIFO_DATA_OUT_H)\n");
        return false;
    }

    return true;
}

void AP_InertialSensor_LSM6DS33::_read_fifo()
{
    struct sensor_raw_data raw_data[1365] = {0};

    // 1. Read the FIFO_STATUS1 and FIFO_STATUS2 registers to check how many words
    // (16-bit data) are stored in the FIFO. This information is contained in the
    // DIFF_FIFO_[11:0] bits.
    uint16_t num_samples = 0;
    bool status = _dev->read_registers(FIFO_STATUS1, (uint8_t *) &num_samples, 2);
    if (!status) {
        DEV_PRINTF("LSM6DS33: Unable to read the number of samples in the FIFO (FIFO_STATUS1, FIFO_STATUS2)\n");
        return;
    }
    num_samples &= 0xFFF;  // Consider only DIFF_FIFO_[11:0] bits.
//    hal.console->printf("num samples: %d\n", num_samples);

    // 2. Read the FIFO_STATUS3 and FIFO_STATUS4 registers. The FIFO_PATTERN_[9:0]
    // bits allows understanding which sensor and which couple of bytes is being read.
    // TODO: write asserts

    // 3. Read the FIFO_DATA_OUT_L and FIFO_DATA_OUT_H registers to retrieve the oldest
    // sample (16-bits format) in the FIFO. They are respectively the lower and the upper part
    // of the oldest sample.
    status = _dev->read_registers(FIFO_DATA_OUT_L, (uint8_t *)raw_data, 2 * num_samples);
    if (!status) {
        DEV_PRINTF("LSM6DS33: Unable to read FIFO data (FIFO_DATA_OUT_L, FIFO_DATA_OUT_H)\n");
        return;
    }

    uint16_t num_data = num_samples / 3;  // Each reading consists of 3 16-bit values (x, y, z)
    for (uint16_t i = 0; i < num_data; i += 2) {
        Vector3f gyro{(float)(int16_t)le16toh(raw_data[i].x),
                      (float)(int16_t)le16toh(raw_data[i].y),
                      (float)(int16_t)le16toh(raw_data[i].z)};
        Vector3f accel{(float)(int16_t)le16toh(raw_data[i + 1].x),
                       (float)(int16_t)le16toh(raw_data[i + 1].y),
                       (float)(int16_t)le16toh(raw_data[i + 1].z)};

//        hal.console->printf("gyro: %02x:%02x:%02x\n", raw_data[i].x, raw_data[i].y, -raw_data[i].z);
//        hal.console->printf("acc: %02x:%02x:%02x\n", raw_data[i + 1].x, raw_data[i + 1].y, -raw_data[i + 1].z);

        accel.rotate(_rotation);
        gyro.rotate(_rotation);

        accel *= _accel_scale;
        gyro *= _gyro_scale;

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);
    }
}

#endif

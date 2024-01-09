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
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/I2CDevice.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LSM6DS33 : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_LSM6DS33(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    
    
    
    virtual ~AP_InertialSensor_LSM6DS33();

    // probe the sensor on I2C bus
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

   
   
    /* update accel and gyro state */
    bool update() override;

    void start(void) override;

    
private:

    enum gyro_scale {
        G_SCALE_245DPS = 0,
        G_SCALE_500DPS,
        G_SCALE_2000DPS,
    };

    enum accel_scale {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_6G,
        A_SCALE_8G,
        A_SCALE_16G,
    };

    bool _accel_data_ready();
    bool _gyro_data_ready();
    bool _accel_gyro_init();
    bool _init_sensor();
    void _accumulate_gyro();
    void _accumulate_accel();
    void _set_gyro_scale(gyro_scale scale);
    void _set_accel_scale(accel_scale scale);

    struct PACKED sensor_raw_data {
            int16_t x;
            int16_t y;
            int16_t z;
    };


    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    void _set_filter_frequency(uint8_t filter_hz);

    // gyro and accel instances
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
    float _gyro_scale;
    float _accel_scale;
};
#endif // __AP_INERTIAL_SENSOR_L3G4200D2_H__
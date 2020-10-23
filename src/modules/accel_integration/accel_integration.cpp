/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file accel_integration.cpp
 * Minimal module example for integrating acceleration readings over time.
 *
 * @author Shantanu Wadnerkar <shantanu5996@gmail.com>
 */


#include "accel_integration.h"


AccelIntegration::AccelIntegration()
    : ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    PX4_INFO("Constructing AccelIntegration object..");
}


AccelIntegration::~AccelIntegration()
{
    PX4_INFO("Destructing AccelIntegration object...");
}


int AccelIntegration::task_spawn(int argc, char *argv[])
{
    PX4_INFO("Inside task_spawn");

    AccelIntegration *instance = new AccelIntegration();

	if (instance)
    {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init())
        {
			return PX4_OK;
		}

	}
    else
    {
		PX4_ERR("Instance allocation failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int AccelIntegration::custom_command(int argc, char *argv[])
{
    return print_usage("Unknown command");
}


int AccelIntegration::print_usage(const char *reason)
{
    PX4_INFO("Inside print_usage");

    if (reason)
    {
        PX4_WARN("%s\n", reason);
    }
    return 0;
}


int AccelIntegration::print_status()
{
    PX4_INFO("Running...");
    return 0;
}


void AccelIntegration::Run()
{
    vehicle_status_s status; // Status of UAV, armed or disarmed
    sensor_combined_s sensor_combined_data; // Accelerometer data
    integrated_accel_s velocity_data; // Velocity data to be pbulished

    // Unregister callback if the module is stopped or PX4 shutdown
    if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

    // If status changes, update it here.
    if (_status_sub.update(&status))
    {
        if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
            {
            _armed = true;
            }
        else
        {
            _armed = false; // if this statement if kept above the status update condition, rate of sensor data msg update becomes slow
        }
    }

    // Integrate accelleration only if it is armed and if there is change in reading. Status and Sensor topics publish at unequal rates. That's why they are divided in separate conditions
    if (_armed && _sensor_combined_sub.update(&sensor_combined_data)) {
        double delta_vel_dt{sensor_combined_data.accelerometer_integral_dt * 1.e-6f};
        _vel_x += (double)sensor_combined_data.accelerometer_m_s2[0] * delta_vel_dt;
        _vel_y += (double)sensor_combined_data.accelerometer_m_s2[1] * delta_vel_dt;
        _vel_z += (double)sensor_combined_data.accelerometer_m_s2[2] * delta_vel_dt;

        velocity_data.timestamp = (double)sensor_combined_data.timestamp;
        velocity_data.integral_dt = (double)sensor_combined_data.accelerometer_integral_dt;
        velocity_data.vel_x = _vel_x;
        velocity_data.vel_y = _vel_y;
        velocity_data.vel_z = _vel_z;

        _integrated_accel_pub.publish(velocity_data);
    }
}


bool AccelIntegration::init()
{
    PX4_INFO("Initialisating object...");

	if (!_sensor_combined_sub.registerCallback()) {
		PX4_ERR("Sensor_combined callback registration failed!");
		return false;
	}

	return true;
}


int accel_integration_main(int argc, char *argv[])
{
    PX4_INFO("Running ACCEL_INTEGRATION module...");
    
    return AccelIntegration::main(argc, argv);
}

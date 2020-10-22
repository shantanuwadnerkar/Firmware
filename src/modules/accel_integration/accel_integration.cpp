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
 * Minimal application example for PX4 autopilot
 *
 * @author Shantanu Wadnerkar <shantanu5996@gmail.com>
 */


#include "accel_integration.h"


AccelIntegration::AccelIntegration()
    : ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
    PX4_INFO("Inside AccelIntegration constructor");
}


AccelIntegration::~AccelIntegration()
{
    PX4_INFO("Inside AccelIntegration destructor");
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
    PX4_INFO("Inside custom_command");
    return 0;
}


int AccelIntegration::print_usage(const char *reason)
{
    PX4_INFO("Inside print_usage");
    return 0;
}


int AccelIntegration::print_status()
{
    PX4_INFO("Inside print_status");
    return 0;
}


void AccelIntegration::Run()
{
    PX4_INFO("Inside run");

    if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

    
    sensor_combined_s sensors;

    if (_sensor_combined_sub.update(&sensors)) {
        std::cout << "sensor msg update" << "\t";
    }

    std::cout << &sensors << "\n";

    while (!should_exit())
    {
        // PX4_INFO("Inside run - inside loop");
    }
}


bool AccelIntegration::init()
{
    PX4_INFO("Inside init");

	if (!_sensor_combined_sub.registerCallback()) {
		PX4_ERR("sensor combined callback registration failed!");
		return false;
	}

	return true;
}


int accel_integration_main(int argc, char *argv[])
{
    PX4_INFO("Running ACCEL_INTEGRATION App...");

    // uORB::Subscription sc(ORB_ID(sensor_combined), 1, 0);
    // int abc = orb_subscribe(ORB_ID(sensor_combined));
    // std::cout << abc << "\n";
    
    return AccelIntegration::main(argc, argv);
}

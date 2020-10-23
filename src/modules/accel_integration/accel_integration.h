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


#pragma once

#include <iostream>

#include <px4_log.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <perf/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/integrated_accel.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_status.h>


extern "C" __EXPORT int accel_integration_main(int argc, char* argv[]);

class AccelIntegration : public ModuleBase<AccelIntegration>, public ModuleParams, public px4::WorkItem
{
public:
    AccelIntegration();

    ~AccelIntegration() override;

	/** @see ModuleBase 
	 * Starter function. This function is called after AccelIntegration::main() is done.
	*/
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase 
	 * Print help.
	*/
	static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::print_status() 
	 * Print the current status of the module.
	*/
	int print_status() override;

	/** @see ModuleBase::run() */
	void Run() override;

    bool init();

private:
	/**
	 * Variable to check if the UAV is armed or not.
	*/
	bool _armed{false};

	/**
	 * Variables to store the summation of velocity in x, y, z axes.
	*/
	double _vel_x{0.0};
	double _vel_y{0.0};
	double _vel_z{0.0};

    // Subscriptions
	uORB::SubscriptionCallbackWorkItem _status_sub{this, ORB_ID(vehicle_status)}; // Status of UAV to check if it's armed or disarmed
	uORB::SubscriptionCallbackWorkItem	_sensor_combined_sub{this, ORB_ID(sensor_combined)}; // uORB topic referenced in EKF2 to subscribe to accelerometer readings
	
	// Publication
	uORB::Publication<integrated_accel_s> _integrated_accel_pub{ORB_ID(integrated_accel)}; // Publish the integrated delta acceleration or velocity for X, Y, Z axes
};

/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskLand.cpp
 */

#include "FlightTaskLand.hpp"

bool
FlightTaskLand::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	Vector3f vel_prev{last_setpoint.velocity};
	Vector3f pos_prev{last_setpoint.position};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current position
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// If no acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }


	}

	_yaw_setpoint  = _land_heading = _yaw; // set the yaw setpoint to the current yaw
	_position_smoothing.reset(pos_prev, vel_prev, accel_prev);


	_acceleration_setpoint = accel_prev;
	_velocity_setpoint = vel_prev;
	_position_setpoint = pos_prev;


	// Initialize the Landing locations and parameters

	// calculate where to land based on the current velocity and acceleration constraints
	// set this as the target location for position smoothing
	_updateTrajConstraints();

	_CalculateBrakingLocation();
	_initial_land_position = _land_position;

	return ret;
}

void
FlightTaskLand::reActivate()
{
	FlightTask::reActivate();
	PX4_ERR("FlightTaskLand reActivate was called!");
	// On ground, reset acceleration and velocity to zero
	_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.7f}, _position);
}

bool
FlightTaskLand::update()
{
	bool ret = FlightTask::update();

	if (!_is_initialized) {
		PX4_ERR("initializing _position_smoothing");
		PX4_ERR("_velocity_setpoint: %f, %f, %f", (double)_velocity_setpoint(0), (double)_velocity_setpoint(1),
			(double)_velocity_setpoint(2));
		PX4_ERR("_position_setpoint: %f, %f, %f", (double)_position_setpoint(0), (double)_position_setpoint(1),
			(double)_position_setpoint(2));
		PX4_ERR("_acceleration_setpoint: %f, %f, %f", (double)_acceleration_setpoint(0), (double)_acceleration_setpoint(1),
			(double)_acceleration_setpoint(2));
		_position_smoothing.reset(_acceleration_setpoint, _velocity, _position);
		_is_initialized = true;
	}

	PX4_WARN("_position: 		%f, 	%f, 	%f", (double)_position(0), (double)_position(1), (double)_position(2));
	PX4_WARN("_land_position: 		%f, 	%f, 	%f", (double)_land_position(0), (double)_land_position(1),
		 (double)_land_position(2));

	if (_landing) {
		_PerformLanding();

	} else {
		_CalculateBrakingLocation();
		_SmoothBrakingPath();
	}

	return ret;
}

void
FlightTaskLand::_PerformLanding()
{
	PX4_ERR("Landing: Vehicle is moving, perform landing");
	// Perform 3 phase landing
	_velocity_setpoint.setNaN();

	// Calculate the vertical speed based on the distance to the ground
	float vertical_speed = math::interpolate(_dist_to_ground,
			       _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
			       _param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get());

	bool range_dist_available = PX4_ISFINITE(_dist_to_bottom);

	// If we are below the third phase altitude, use the crawl speed
	if (range_dist_available && _dist_to_bottom <= _param_mpc_land_alt3.get()) {
		vertical_speed = _param_mpc_land_crawl_speed.get();
	}

	_position_setpoint = {_land_position(0), _land_position(1), NAN}; // The last element of the land position has to stay NAN
	_yaw_setpoint = _land_heading;
	_velocity_setpoint(2) = vertical_speed;
	_gear.landing_gear = landing_gear_s::GEAR_DOWN;

}
void
FlightTaskLand::_SmoothBrakingPath()
{
	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	_position_smoothing.generateSetpoints(
		_position,
		_land_position,
	{0.f, 0.f, 0.f},
	_deltatime,
	false,
	out_setpoints
	);

	_jerk_setpoint = out_setpoints.jerk;
	_acceleration_setpoint = out_setpoints.acceleration;
	_velocity_setpoint = out_setpoints.velocity;
	_position_setpoint = out_setpoints.position;
	_yaw_setpoint = _land_heading;
	PX4_WARN("_position_setpoint: 	%f, 	%f, 	%f", (double)_position_setpoint(0), (double)_position_setpoint(1),
		 (double)_position_setpoint(2));

	if (_velocity.xy().norm() <
	    _param_nav_mc_alt_rad.get()) { //} || _velocity(2) < _param_mpc_z_v_auto_dn.get()){ // not sure about the last parts, check!)
		_landing = true;
		PX4_WARN("Landing: Vehicle is not moving, perform landing");
	}

}

void
FlightTaskLand::_CalculateBrakingLocation()
{
	// Calculate the 3D point where we until where we can slow down smoothly and then land based on the current velocities and system constraints on jerk and acceleration.

	float delay_scale = 0.6f; // delay scale factor
	const float velocity_hor_abs = sqrtf(_velocity(0) * _velocity(0) + _velocity(1) * _velocity(1));
	const float braking_dist_xy = math::trajectory::computeBrakingDistanceFromVelocity(velocity_hor_abs,
				      _param_mpc_jerk_auto.get(), _param_mpc_acc_hor.get(), delay_scale * _param_mpc_jerk_auto.get());
	PX4_WARN("Braking distance: %f", (double)braking_dist_xy);
	float braking_dist_z = 0.0f;

	if (_velocity(2) < -0.1f) {
		PX4_WARN("Moving upwards: %f", (double)_velocity(2));
		braking_dist_z = math::trajectory::computeBrakingDistanceFromVelocity(_velocity(2),
				 _param_mpc_jerk_auto.get(), _param_mpc_acc_down_max.get(), delay_scale * _param_mpc_jerk_auto.get());

	} else if (_velocity(2) > 0.1f) {
		PX4_WARN("Moving downwards");
		braking_dist_z = math::trajectory::computeBrakingDistanceFromVelocity(_velocity(2),
				 _param_mpc_jerk_auto.get(), _param_mpc_acc_up_max.get(), delay_scale * _param_mpc_jerk_auto.get());
	}

	const Vector3f braking_dir = _velocity.unit_or_zero();
	const Vector3f braking_dist = {braking_dist_xy, braking_dist_xy, braking_dist_z};
	Vector3f Temp = braking_dir.emult(braking_dist);
	PX4_WARN("Braking distance: %f, %f, %f", (double)Temp(0), (double)Temp(1), (double)Temp(2));
	_land_position = _position + braking_dir.emult(braking_dist);
	PX4_INFO("FlightTaskMyTask CalculateBrakingLocation was called!"); // report calculation
}

void
FlightTaskLand::_updateTrajConstraints()
{
	// update params of the position smoothing
	_position_smoothing.setCruiseSpeed(_param_mpc_xy_vel_max.get());
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setTargetAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());

	// Update the constraints of the trajectories
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get());
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	_position_smoothing.setMaxJerk(_param_mpc_jerk_auto.get());

	// set the constraints for the vertical direction
	_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
	_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
	// if moving up, acceleration constraint is always in deceleration direction, eg opposite to the velocity
	// if (_velocity(2) < 0.0f && !_landing) {
	// 	_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
	// 	_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_up.get());

	// } else if (!_landing) {
	// 	_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_up_max.get());
	// 	_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());

	// } else {
	// 	_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
	// 	_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
	// }

	// should the constraints be different when switching from an auto mode compared to an manual mode?
}

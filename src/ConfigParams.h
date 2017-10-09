#pragma once

/**
 * Collects parameters governing the simulation and the driving. All measures use
 * the I.S. for measurement units.
 */
namespace ConfigParams {
	// Yes, this is the lane width!
	static constexpr double lane_width= 4.;

	// The planning time interval; when planning a path, it is planned to be worth these many seconds
	constexpr double planning_t = 1;

	// Desired cruise speed, that the behaviour planning shall try to attain and keep when possible
	constexpr double cruise_speed = 21.4579;

			// Maximum acceptable acceleration for the car, the car will try to reach it in order to get to target_sped in the shortest time
	constexpr double max_accel_s = 7;

	// Time interval between two consecutive waypoints, as implemented by the simulator
	constexpr double tick = 0.02;

	// When the planned trajectory yet to be run goes under this duration, extend it by planning a new trajectory
	constexpr double min_trajectory_duration = 0.5;

	// Simulated sensors range; vehicles that are reported by the simulator outside this range are ignored
	constexpr double sensor_range= 300;

	// The length of the the track and the max value of s in Frenet coordinates
	constexpr double max_s = 6945.554;
}

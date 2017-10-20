#pragma once
#include <random>

/**
 * Collects parameters governing the simulation and the driving. All measures use
 * the I.S. for measurement units.
 */
namespace ConfigParams {
	// Yes, this is the lane width!
	static constexpr double lane_width= 4.;

	// The planning time interval; when planning a path, it is planned to be worth these many seconds
	constexpr double planning_t_KL = 2.3;  // Used when in KeepLane and FollowCar states
	constexpr double planning_t_CL = 1.8;  // Used when in ChangeLane state

	// Desired cruise speed, that the behaviour planning shall try to attain and keep when possible
	constexpr double cruise_speed = 20.78736;  // 46.5 mph

	// Speed limit
	constexpr double speed_limit= 22.35;

	// Desired safe distance between the car and another car preceding in the same lane
	constexpr double safe_distance= 2.5*cruise_speed;

	// Desired safe distance from following/preceding vehicle in another lane, before starting lane change
	constexpr double overtaking_margin_following= ConfigParams::safe_distance/4;
	constexpr double overtaking_margin_preceding= ConfigParams::safe_distance*.8;

	// Maximum acceptable acceleration for the car, the car will try to reach it in order to get to target_sped in the shortest time
	constexpr double max_accel_s = 5;

	// Time interval between two consecutive waypoints, as implemented by the simulator
	constexpr double tick = 0.02;

	// When the planned trajectory yet to be run goes under this duration, extend it by planning a new trajectory
	constexpr double min_trajectory_duration = 0.1;

	// Simulated sensors range; vehicles that are reported by the simulator outside this range are ignored
	constexpr double sensor_range= 300;

	// The length of the the track and the max value of s in Frenet coordinates
	constexpr double max_s = 6945.554;

	// Number of trajectory variations from where to sample the next trajectory
	constexpr unsigned n_trajectories= 50;

	// The random number generator
	static std::default_random_engine rng;

	/* Set the size of the interval from where the goal of trajectories is sampled, as
	 * a fraction of the goal-start distance along the s coordinate.
	 */
	constexpr double sampling_interval= 1./6.;
}

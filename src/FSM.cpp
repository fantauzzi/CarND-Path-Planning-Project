#include <climits>
#include <utility>
#include <iostream>
#include <vector>
#include <cmath>
#include "Eigen/Core"
#include "FSM.h"
#include "Car.h"
#include "trajectory.h"
#include "ConfigParams.h"

using namespace std;
using namespace Eigen;

pair<double, double> findClosestInLane(Coordinates sd,  vector<CarSensorData> cars, unsigned lane, bool preceding, double lane_width ) {
	int closest_i= -1;  // Will be the position in cars[] of the found vehicle (if found)
	double closest_dist=  numeric_limits<double>::max();
	int sign= (preceding)? 1: -1;
	for (unsigned i=0; i<cars.size(); ++i)  {
		double separation = -sign*cars[i].measureSeparationFrom(sd.first);
		if (separation >=0 && separation < closest_dist && abs(cars[i].d-lane_width/2-lane_width*lane) < lane_width/2) {
			closest_dist= separation;
			closest_i=i;
		}
	}
	return {closest_i, closest_dist };
}


FSM_State::FSM_State(const Car car_init, const std::vector<CarSensorData> cars_init):
	car(car_init), cars(cars_init), boundary_conditions_initialised(false) {
}

void FSM_State::initBoundaryConditions(Eigen::Vector3d s_init, Eigen::Vector3d d_init) {
	last_s_boundary_conditions= s_init;
	last_d_boundary_conditions= d_init;
	boundary_conditions_initialised= true;
}

KeepLane::KeepLane(const Car & car_init, const std::vector<CarSensorData> cars_init):
		FSM_State(car_init, cars_init), target_speed(.0) {
}

FSM_State * KeepLane::getNextState(const Car & theCar, const std::vector<CarSensorData> theCars) {
	car= theCar;
	cars= theCars;
	// Find the closest vehicle in range preceding in the same lane (if any)
	auto closest_info= findClosestInLane({car.s, car.d}, cars, car.getLane(), true, ConfigParams::lane_width);
	int closest_i= closest_info.first;  // Will be the position in cars[] of the found vehicle (if found)
	double closest_dist=  closest_info.second;

	/* If the distance is below a certain amount, set the target speed to the speed
	 * of the preceding car, less some margin; otherwise set the target speed to the max cruise speed
	 */
	bool consider_lane_change= false;
	if (closest_i >= 0 && closest_dist <= 50) {
		target_speed= min(sqrt(pow(cars[closest_i].vx,2)+pow(cars[closest_i].vy,2))*.99, ConfigParams::cruise_speed);
		consider_lane_change= true;  // TODO not quite right, what if the preceding car is faster than cruise_speed?
	}
	else
		target_speed= ConfigParams::cruise_speed;

	if (consider_lane_change) {
		// cout << "Considering lane change." << endl;
		// Which lanes should we consider for a lane change?
		vector<unsigned> lanes;
		if (car.getLane()==1)
			lanes= {0u, 2u};
		else
			lanes= {1u};
		int new_lane= -1;  // Will be set to the best lane to change to, or left set to -1 if no suitable lane change is found
		double new_lane_speed= 0;

		for (auto the_lane: lanes) {
			// Find closest preceding vehicle (if any) in adjacent lane
			auto preceding = findClosestInLane({car.s, car.d }, cars, the_lane, true, ConfigParams::lane_width);
			int preceding_i= preceding.first;
			double preceding_dist= preceding.second;
			// if (preceding_i >=0)
				// cout << "Found preceding car in lane " << the_lane << " with distance " << preceding_dist << endl;
			auto following = findClosestInLane({car.s, car.d }, cars, the_lane, false, ConfigParams::lane_width);
			int following_i= following.first;
			double following_dist= following.second;
			// if (following_i >=0)
				// cout << "Found following car in lane " << the_lane << " with distance " << following_dist << endl;
			// Is this lane change viable?
			bool viable= true;
			if (following_i >=0 && following_dist < 10)
				viable= false;
			else if (preceding_i >=0 && preceding_dist <15)
				viable= false;
			if (viable && (preceding_i<0 || (preceding_i>=0 && cars[preceding_i].getSpeed() > new_lane_speed))) {
				new_lane= the_lane;
				new_lane_speed= (preceding_i >= 0)? cars[preceding_i].getSpeed() : ConfigParams::cruise_speed;  // TODO this is not used!
			}
		}
		if (new_lane >= 0) {
			cout << "Changing from lane " << car.getLane() << " to lane " << new_lane << endl;
			auto pNextState= new ChangeLane(car, cars, new_lane, new_lane_speed);
			pNextState->initBoundaryConditions(last_s_boundary_conditions, last_d_boundary_conditions);
			return pNextState;
		}
	}
	return this;
}

pair<Vector6d, Vector6d> KeepLane::computeBoundaryConditions() {
	assert(boundary_conditions_initialised);
	Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
	Vector3d s_goal;// Goal conditions for s

	/* Could it be in the base class?
	 * - Compute possible s_goal and d_goal conditions based on kynematic equations, target lane and
	 * planning time interval.
	 * - Randomly perturb the found s_goal and d_goal to generate more candidate trajectories
	 * - Select the trajectory with the lowest cost
	 */

	/*
	 * How to compute possible s_goal and d_goal.
	 * - Assume a linear trajectory from start to goal (its length will be a lower bound
	 * for the actual trajectory length).
	 * - With kynematic equations, determine how much distance the car will cover in the given
	 * planning time.
	 * - Determine the consequent goal_s and goal_d
	 *
	 */

	// Determine the s component of the target speed at the car position
	double road_h = car.converter.getRoadHeading(car.s);  // TODO yuck!
	double target_speed_s= target_speed*cos(car.yaw- road_h);

	int a_sign = (target_speed_s > s_start[1])? 1 : -1;
	double proj_vel_s = s_start[1] + a_sign*ConfigParams::max_accel_s*ConfigParams::planning_t_KL;
	if ((a_sign > 0 && proj_vel_s < target_speed_s) || (a_sign < 0 && proj_vel_s > target_speed_s))
		s_goal << s_start[0]+s_start[1]*ConfigParams::planning_t_KL+.5*(a_sign)*ConfigParams::max_accel_s*pow(ConfigParams::planning_t_KL,2), proj_vel_s, a_sign*ConfigParams::max_accel_s;
	else {
		double tx= a_sign*(target_speed_s - s_start[1]) / ConfigParams::max_accel_s;
		double s1= s_start[0] + s_start[1]*tx+.5*a_sign*ConfigParams::max_accel_s*pow(tx,2);
		double s2= (ConfigParams::planning_t_KL - tx) * target_speed_s;
		s_goal << s1+s2, target_speed_s, 0;
	}
	cout << "KeepLane s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

	Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
	Vector3d d_goal;// Goal conditions for d
	d_goal << ConfigParams::lane_width/2+car.getLane()*ConfigParams::lane_width, 0, 0;
	cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

	// Update the boundary conditions to be used at the beginning of the next JMT
	last_s_boundary_conditions = s_goal;
	if (last_s_boundary_conditions[0] >= ConfigParams::max_s)
		last_s_boundary_conditions[0]-= ConfigParams::max_s;
	last_d_boundary_conditions = d_goal;

	// Compute the quintic polynomial coefficients, for the given boundary conditions and planning time interval
	auto sJMT = computeJMT(s_start, s_goal, ConfigParams::planning_t_KL);
	cout << "sJMT= " << sJMT.transpose() << endl << endl;
	auto dJMT = computeJMT(d_start, d_goal, ConfigParams::planning_t_KL);
	cout << "dJMT= " << dJMT.transpose() << endl << endl;
	return {sJMT, dJMT};
}

ChangeLane::ChangeLane(
		const Car & car_init,
		const std::vector<CarSensorData> cars_init,
		const unsigned target_lane_init,
		const double target_speed_init):
		FSM_State(car_init, cars_init), target_lane(target_lane_init), target_speed(target_speed_init) {
	cout << "Instantiated ChangeLane with target_lane= "<< target_lane_init << " target_speed=" << target_speed_init << endl;
}

FSM_State * ChangeLane::getNextState(const Car & theCar, const std::vector<CarSensorData> theCars) {
	car= theCar;
	cars= theCars;

	// Is the change of lane complete?
	if (abs(car.d-(ConfigParams::lane_width/2+target_lane*ConfigParams::lane_width)) < .2)  { // TODO tune this
		auto pNextState= new KeepLane(car, cars);
		pNextState->initBoundaryConditions(last_s_boundary_conditions, last_d_boundary_conditions);
		return pNextState;
	}
	return this;
}

pair<Vector6d, Vector6d> ChangeLane::computeBoundaryConditions() {  // TODO name is incorrect, as method returns the polynomials, not the boundary conditions

	Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
	Vector3d s_goal;// Goal conditions for s

	double road_h = car.converter.getRoadHeading(car.s);  // TODO yuck!
	double target_speed_s= target_speed*cos(car.yaw- road_h);

	int a_sign = (target_speed_s > s_start[1])? 1 : -1;
	double proj_vel_s = s_start[1] + a_sign*ConfigParams::max_accel_s*ConfigParams::planning_t_CL;
	if ((a_sign > 0 && proj_vel_s < target_speed_s) || (a_sign < 0 && proj_vel_s > target_speed_s))
		s_goal << s_start[0]+s_start[1]*ConfigParams::planning_t_CL+.5*(a_sign)*ConfigParams::max_accel_s*pow(ConfigParams::planning_t_CL,2), proj_vel_s, a_sign*ConfigParams::max_accel_s;
	else {
		double tx= a_sign*(target_speed_s - s_start[1]) / ConfigParams::max_accel_s;
		double s1= s_start[0] + s_start[1]*tx+.5*a_sign*ConfigParams::max_accel_s*pow(tx,2);
		double s2= (ConfigParams::planning_t_CL - tx) * target_speed_s;
		s_goal << s1+s2, target_speed_s, 0;
	}
	cout << "ChangeLane s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

	Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
	Vector3d d_goal;// Goal conditions for d
	d_goal << ConfigParams::lane_width/2+target_lane*ConfigParams::lane_width, 0, 0;
	cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

	// Update the boundary conditions to be used at the beginning of the next JMT
	last_s_boundary_conditions = s_goal;
	if (last_s_boundary_conditions[0] >= ConfigParams::max_s)
		last_s_boundary_conditions[0]-= ConfigParams::max_s;
	last_d_boundary_conditions = d_goal;

	// Compute the quintic polynomial coefficients, for the given boundary conditions and planning time interval
	auto sJMT = computeJMT(s_start, s_goal, ConfigParams::planning_t_CL);
	cout << "sJMT= " << sJMT.transpose() << endl << endl;
	auto dJMT = computeJMT(d_start, d_goal, ConfigParams::planning_t_CL);
	cout << "dJMT= " << dJMT.transpose() << endl << endl;
	return {sJMT, dJMT};
}

/**
 * Determines the goal boundary conditions for a jerk minimising trajectory (JMT). It uses the boundary conditions
 * stored in `last_s_boundary_conditions` and `last_d_boundary_conditions` as the starting boundary conditions for the JMT.
 * @param target_s the s Frenet coordinate of the desired car position at the end of the trajectory (in meters).
 * @param target_d the d Frenet coordinate of the desired car position at the end of the trajectory (in meters).
 * @param target_v the desired car velocity (in m/s) at the end of the trajectory.
 * @param planning_t the time (in seconds) for the car to cover the trajectory.
 * @return
 */
pair<Vector3d, Vector3d> FSM_State::getGoalConditions(const double target_s,
		const double target_d,
		const double target_v,
		const double planning_t) const {
	assert(boundary_conditions_initialised);
	Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
	Vector3d s_goal;// Goal conditions for s

	/* Could it be in the base class?
	 * - Compute possible s_goal and d_goal conditions based on kynematic equations, target lane and
	 * planning time interval.
	 * - Randomly perturb the found s_goal and d_goal to generate more candidate trajectories
	 * - Select the trajectory with the lowest cost
	 */

	/*
	 * How to compute possible s_goal and d_goal.
	 * - Assume a linear trajectory from start to goal (its length will be a lower bound
	 * for the actual trajectory length).
	 * - With kynematic equations, determine how much distance the car will cover in the given
	 * planning time.
	 * - Determine the consequent goal_s and goal_d
	 *
	 */

	Vector3d dummy1, dummy2;

	return { dummy1, dummy2};
}

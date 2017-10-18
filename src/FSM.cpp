#include <climits>
#include <utility>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
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

FollowCar::FollowCar(const Car & car_init, const std::vector<CarSensorData> cars_init):
		FSM_State(car_init, cars_init) {
}


FSM_State * FollowCar::getNextState(const Car & theCar, const std::vector<CarSensorData> theCars) {
	car= theCar;
	cars= theCars;
	// Find the closest vehicle in range preceding in the same lane (if any)
	auto closest_info= findClosestInLane({car.s, car.d}, cars, car.getLane(), true, ConfigParams::lane_width);
	int closest_i= closest_info.first;  // Will be the position in cars[] of the found vehicle (if found)
	double closest_dist=  closest_info.second;

	/* If the distance is at least the computed safe distance plus a margin, then set the next state to KeepLane;
	 * the margin is introduced to prevent constant flipping between KeepLane and FollowCar.
	 * Set the next state to KeepLane also if no preceding car was found.
	 */

	if ((closest_i >= 0 && closest_dist >= ConfigParams::safe_distance*1.6) || closest_i < 0) {
		auto pNextState= new KeepLane(car, cars);
		pNextState->initBoundaryConditions(last_s_boundary_conditions, last_d_boundary_conditions);
		return pNextState;
	}

	// Otherwise, just stay in the current state
	return this;
}


FSM_State * KeepLane::getNextState(const Car & theCar, const std::vector<CarSensorData> theCars) {
	car= theCar;
	cars= theCars;
	// Find the closest vehicle in range preceding in the same lane (if any)
	auto closest_info= findClosestInLane({car.s, car.d}, cars, car.getLane(), true, ConfigParams::lane_width);
	int closest_i= closest_info.first;  // Will be the position in cars[] of the found vehicle (if found)
	double closest_dist=  closest_info.second;

	// If the distance is below the computed safe distance, then set the next state to FollowCar
	if (closest_i >= 0 && closest_dist < ConfigParams::safe_distance) {
		auto pNextState= new FollowCar(car, cars);
		pNextState->initBoundaryConditions(last_s_boundary_conditions, last_d_boundary_conditions);
		return pNextState;
	}

	// Otherwise, just stay in the current state
	return this;
}


Vector6d differentiate (const Vector6d coeffs) {
	Vector6d res;
	for (unsigned i=0; i<5; ++i)
		res[i]=(i+1)*coeffs[i+1];
	res[5]=0;
	return res;
}

/**
 * Determines the cost of a trajectory between time 0 and a given time. The trajectory must be given
 * as two quintic polynomials s(t) and d(t).
 * @param s_coeffs coefficients of the trajectory s component.
 * @param d_coeffs coefficients of the trajectory d component.
 * @param t the duration of the time interval during which to evaluate the trajectory.
 * @return the calculated cost, between 0 and 1, inclusive.
 */
double cost(
		const Vector6d s_coeffs,
		const Vector6d d_coeffs,
		const double time_interval) {
	// Determine max velocity, max acceleration, max jerk, overall lateral jerk and overall longitudinal jerk


	auto s_diff= s_coeffs;
	auto d_diff= d_coeffs;
	constexpr double delta_t=ConfigParams::tick;
	const unsigned n_intervals= static_cast<unsigned>(round(time_interval/delta_t));

	vector<vector<double>> s_tabulated;
	vector<vector<double>> d_tabulated;
	vector<vector<double>> sd_tabulated;

	for (unsigned deg=1; deg<=3; ++deg) {
			s_diff= differentiate(s_diff);
			d_diff= differentiate(d_diff);
			vector<double> s_values(n_intervals+1, .0);
			vector<double> d_values(n_intervals+1, .0);
			vector<double> sd_values(n_intervals+1, .0);
			for (unsigned i=0; i<=n_intervals; ++i) {
				const double t= i*delta_t;
				s_values[i]=abs(evalQuintic(s_diff, t));
				d_values[i]=abs(evalQuintic(d_diff, t));
				sd_values[i]=sqrt(pow(s_values[i],2)+pow(d_values[i],2));
			}
			s_tabulated.push_back(std::move(s_values));
			d_tabulated.push_back(std::move(d_values));
			sd_tabulated.push_back(std::move(sd_values));
	}
	vector<double> s_max(3, .0);
	vector<double> d_max(3, .0);
	vector<double> sd_max(3, .0);

	for (unsigned i=0; i<3; ++i) {
		s_max[i]= *max_element(s_tabulated[i].begin(), s_tabulated[i].end());
		d_max[i]= *max_element(begin(d_tabulated[i]), end(d_tabulated[i]));
		sd_max[i]= *max_element(begin(sd_tabulated[i]), end(sd_tabulated[i]));
	}

	/*
	if (sd_max[1]>=10 || sd_max[2] >=10 || sd_max[0]>=22.35)
		return 1.;
	else
		return .0;*/
	const double speed_cost = (sd_max[0] > ConfigParams::speed_limit)? 20: 0;
	return sd_max[1]+sd_max[2]+speed_cost;

}

pair<Vector6d, Vector6d> FSM_State::generateTrajectory() {
	/* Compute a basic trajectory first, then generate variations and choose the one with lowest cost
	 *
	 */

	// Determine the boundary conditions for s and d for the JMT

	Vector3d s_start= last_s_boundary_conditions;// Initial conditions for s
	Vector3d d_start= last_d_boundary_conditions;// Initial conditions for s
	auto b_cond= computeGoalBoundaryConditions();
	Vector3d s_goal= b_cond.first;
	Vector3d d_goal= b_cond.second;

	double dist= s_goal[0]-s_start[0];
	double a_over_dist= 2/pow(getPlanningTime(),2)*(dist-s_start[1]*getPlanningTime());
	if (abs(a_over_dist) > ConfigParams::max_accel_s)
		cout << "***** Planned acc. over max: " << a_over_dist << endl;


	// Compute the quintic polynomial (JMT) coefficients, for the given boundary conditions and planning time interval

	// Generate additional random boundary conditions and the corresponding JMTs

	vector<Vector3d> s_goal_variations;
	vector<Vector6d> sJMT_variations;
	vector<Vector6d> dJMT_variations;

	s_goal_variations.push_back(s_goal);
	auto sJMT = computeJMT(s_start, s_goal, getPlanningTime());
	auto dJMT = computeJMT(d_start, d_goal, getPlanningTime());
	sJMT_variations.push_back(sJMT);
	dJMT_variations.push_back(dJMT);
	cout << "sJMT= " << sJMT.transpose() << endl << endl;
	cout << "dJMT= " << dJMT.transpose() << endl << endl;

	normal_distribution<double> distribution(s_goal[0], (s_goal[0] - s_start[0])/4);

	for (unsigned i=0; i< ConfigParams::n_trajectories-1; ++i) {
		const double s_variation= distribution(ConfigParams::rng);
		Vector3d s_varied;
		s_varied << s_variation, s_goal[1], s_goal[2];
		s_goal_variations.push_back(s_varied);
		auto sJMT_varied = computeJMT(s_start, s_varied, getPlanningTime());
		// auto dJMT_varied = computeJMT(d_start, d_goal, getPlanningTime());
		sJMT_variations.push_back(sJMT_varied);
		dJMT_variations.push_back(dJMT);
	}

	// Determine their costs
	// Choose the JMT with minimum cost
	double min_cost= numeric_limits<double>::max();
	unsigned min_cost_i;

	for (unsigned i=0; i<ConfigParams::n_trajectories; ++i) {
		const double theCost=cost(sJMT_variations[i], dJMT_variations[i], getPlanningTime());
		if (theCost < min_cost) {
			min_cost= theCost;
			min_cost_i= i;
		}
	}

	cout << "Min cost JMT is # " << min_cost_i << " with cost= " << min_cost << endl;

	// Update the boundary conditions to be used at the next iteration
	last_s_boundary_conditions = s_goal_variations[min_cost_i];
	if (last_s_boundary_conditions[0] >= ConfigParams::max_s)
		last_s_boundary_conditions[0]-= ConfigParams::max_s;
	last_d_boundary_conditions = d_goal;

	return {sJMT_variations[min_cost_i], dJMT_variations[min_cost_i]};
}

pair<Vector3d, Vector3d> FollowCar::computeGoalBoundaryConditions() {
	assert(boundary_conditions_initialised);
	Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
	Vector3d s_goal;// Goal conditions for s

	auto closest_info= findClosestInLane({car.s, car.d}, cars, car.getLane(), true, ConfigParams::lane_width);
	int closest_i= closest_info.first;  // Will be the position in cars[] of the found vehicle (if found)
	assert(closest_i >= 0);

	const CarSensorData & preceding= cars[closest_i];

	// Estimate s coordinate of the preceding car at the end of the next planning interval (trajectory), assuming constant speed
	const double prec_car_v= sqrt(pow(preceding.vx,2)+pow(preceding.vy,2)); // TODO move it to a memeber function for CarSensorData
	const double time_to_trajectory_end= car.path_x.size()*ConfigParams::tick;
	const double prec_car_s_est= preceding.s+prec_car_v*(time_to_trajectory_end + ConfigParams::planning_t_KL);  // TODO should I give it its own planning time?

	// Determine wanted s coordinate for this car at the end of the next planning interval
	const double wanted_s= prec_car_s_est - ConfigParams::safe_distance;

	// Check if it is possible to get to that s coordinate at that time without violating acceleration constraints
	const double delta_s= wanted_s - s_start[0];
	const double delta_t= ConfigParams::planning_t_KL;
	const double a_to_wanted_s= 2 / pow(delta_t,2) *(delta_s-s_start[1]*delta_t);
	if (delta_s <=0 || abs(a_to_wanted_s) > ConfigParams::max_accel_s) {
		const double a_sign= (a_to_wanted_s > 0 && delta_s > 0)? 1: -1;
		const double s_with_max_a= s_start[0]+s_start[1]*delta_t+.5*a_sign*ConfigParams::max_accel_s*pow(delta_t,2);
		const double v_with_max_a= s_start[1]+a_sign*ConfigParams::max_accel_s*delta_t;
		s_goal << s_with_max_a, min(v_with_max_a, ConfigParams::cruise_speed), a_sign*ConfigParams::max_accel_s;
	}
	else
		s_goal << prec_car_s_est - ConfigParams::safe_distance, min(ConfigParams::cruise_speed,prec_car_v), 0;

	// Don't allow to go backward!
	if (s_goal[0] < s_start[0]) {
		cout << "*** Got s goal before start: " <<s_goal[0] << " " << s_start[0] << endl;
		s_goal << s_start[0], 0, 0;
	}
	// Check if it is possible to cover the trajectory distance without violating the speed limit
	double avg_speed= (s_goal[0] - s_start[0]) / ConfigParams::planning_t_KL;
	if (avg_speed > ConfigParams::cruise_speed) {
		// If not, then get s_goal[0] close enough to make it possible
		s_goal[0]= s_start[0] + ConfigParams::cruise_speed*ConfigParams::planning_t_KL;
		s_goal[1]= ConfigParams::cruise_speed;
	}
	cout << "FollowCar s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

	Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
	Vector3d d_goal;// Goal conditions for d
	d_goal << ConfigParams::lane_width/2+car.getLane()*ConfigParams::lane_width, 0, 0;
	cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

	return { s_goal, d_goal };
}

pair<Vector3d, Vector3d> KeepLane::computeGoalBoundaryConditions() {
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

	// Determine the s component of the cruise speed at the car position
	double road_h = car.converter.getRoadHeading(car.s);  // TODO yuck!
	double target_speed_s= ConfigParams::cruise_speed*cos(car.yaw- road_h);

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

	return { s_goal, d_goal };
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

pair<Vector3d, Vector3d> ChangeLane::computeGoalBoundaryConditions() {  // TODO name is incorrect, as method returns the polynomials, not the boundary conditions

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
	// return {sJMT, dJMT};
	return {s_goal, d_goal};
}


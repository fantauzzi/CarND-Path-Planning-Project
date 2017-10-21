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

bool close_enough(const double a, const double b) {
	constexpr double tollerance = 0.001;
	if (abs(a - b) <= tollerance)
		return true;
	return false;
}

double d_forLane(const unsigned lane) {
	switch(lane) {
	case 0:
		return 2.1;
	case 1:
		return 6;
	case 2:
		return 9.8;  // Instead of 10, because of inaccuracies in Frenet/Cartesian conversion
	default:
		assert(0);
	}

	return 0;  // Unreachable

}

pair<double, double> findClosestInLane(Coordinates sd,  vector<CarSensorData> cars, unsigned lane, bool preceding, double lane_width ) {
	int closest_i= -1;  // Will be the position in cars[] of the found vehicle (if found)
	double closest_dist=  numeric_limits<double>::max();
	int sign= (preceding)? 1: -1;
	for (unsigned i=0; i<cars.size(); ++i)  {
		double separation = -sign*measureSeparation(cars[i].s, sd.first);
		if (separation >=0 && separation < closest_dist && abs(cars[i].d-lane_width/2-lane_width*lane) < lane_width/2) { // TODO improve lane calculation
			closest_dist= separation;
			closest_i=i;
		}
	}
	return {closest_i, closest_dist };
}

double predictSeparation(const Car & this_car, const CarSensorData & other_car, const double delta_t) {
	double sep= measureSeparation(this_car.s+this_car.speed*delta_t, other_car.s+other_car.getSpeed()*delta_t);
	return sep;
}

Vector6d differentiate (const Vector6d coeffs) {
	Vector6d res;
	for (unsigned i=0; i<5; ++i)
		res[i]=(i+1)*coeffs[i+1];
	res[5]=0;
	return res;
}

double logistic(const double x) {
	return 1/(1+exp(-x));
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
	// Will store the coefficients of subsequent derivatives of the trajectories
	auto s_diff= s_coeffs;
	auto d_diff= d_coeffs;

	// A few shorthands
	constexpr double delta_t=ConfigParams::tick;
	const unsigned n_intervals= static_cast<unsigned>(round(time_interval/delta_t));

	// Will hold information about trajectory variations
	// vector<vector<double>> s_tabulated;
	// vector<vector<double>> d_tabulated;
	vector<vector<double>> sd_tabulated;  // the modulus of the vector (s, d)
	vector<double> s_vel_tabulated; // the velocity of s (with sign)

	for (unsigned deg=1; deg<=3; ++deg) {
			s_diff= differentiate(s_diff);
			d_diff= differentiate(d_diff);
			vector<double> sd_values(n_intervals+1, .0);
			vector<double> s_vel_values(n_intervals+1, .0);
			for (unsigned i=0; i<=n_intervals; ++i) {
				const double t= i*delta_t;
				const double s_value= abs(evalQuintic(s_diff, t));
				const double d_value= abs(evalQuintic(d_diff, t));
				sd_values[i]=sqrt(pow(s_value,2)+pow(d_value,2));
				if (deg==1)
					s_vel_values[i]= s_value;
			}
			sd_tabulated.push_back(std::move(sd_values));
			if (deg==1)
				s_vel_tabulated= std::move(s_vel_values);
	}
	vector<double> sd_max(3, .0);

	for (unsigned i=0; i<3; ++i) {
		sd_max[i]= *max_element(begin(sd_tabulated[i]), end(sd_tabulated[i]));
	}

	const double s_vel_min= *min_element(begin(s_vel_tabulated), end(s_vel_tabulated));

	const double acc_cost= logistic((sd_max[1]-9)*5)*100;
	const double jerk_cost= logistic((sd_max[2]-9)*5)*100;
	const double speed_cost = (sd_max[0] > ConfigParams::speed_limit*.98)? 500: 0;
	const double neg_s_vel_cost = (s_vel_min <=0)? 1000: 0;

	// cout << "Costs: acc=" << acc_cost << " jerk=" << jerk_cost << " speed=" << speed_cost << " neg vel=" << neg_s_vel_cost << endl;

	return acc_cost+jerk_cost+speed_cost+neg_s_vel_cost;

}

FSM_State::FSM_State(const Car car_init, const std::vector<CarSensorData> cars_init):
	car(car_init), cars(cars_init), boundary_conditions_initialised(false) {
}

void FSM_State::initBoundaryConditions(Eigen::Vector3d s_init, Eigen::Vector3d d_init) {
	last_s_boundary_conditions= s_init;
	last_d_boundary_conditions= d_init;
	boundary_conditions_initialised= true;
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
	if (abs(a_over_dist) > ConfigParams::max_accel_s and ! close_enough(abs(a_over_dist), ConfigParams::max_accel_s))
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

	double sd_dist=0;
	double xy_dist=0;
	double prev_s= evalQuintic(sJMT, 0);
	double prev_d= evalQuintic(dJMT, 0);
	auto xy= car.converter.getXY(prev_s, prev_d);
	double prev_x= xy.first;
	double prev_y= xy.second;
	for (double t=ConfigParams::tick/2; t<=getPlanningTime(); t+=ConfigParams::tick/2)
	{
		double s= evalQuintic(sJMT, t);
		double d= evalQuintic(dJMT, t);
		auto xy= car.converter.getXY(s, d);
		double x= xy.first;
		double y= xy.second;
		sd_dist+=sqrt(pow(s-prev_s,2)+pow(d-prev_d,2));
		xy_dist+=sqrt(pow(x-prev_x,2)+pow(y-prev_y,2));
		prev_s=s;
		prev_d=d;
		prev_x=x;
		prev_y=y;
	}

	double dist_ratio= sd_dist/xy_dist;
	cout << "Distance ratio Frenet/Cartesian= " << dist_ratio << endl;

	sJMT[0]*= dist_ratio;

	const double half_interval= (abs(s_goal[0] - s_start[0]))*(2*ConfigParams::sampling_interval);
	uniform_real_distribution<double> distribution(s_goal[0]-half_interval, s_goal[0]+half_interval);

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

	if ((closest_i >= 0 && closest_dist >= ConfigParams::safe_distance*1.33) || closest_i < 0) {
		auto pNextState= new KeepLane(car, cars);
		pNextState->initBoundaryConditions(last_s_boundary_conditions, last_d_boundary_conditions);
		return pNextState;
	}

	// Otherwise evaluate whether to change lane

	// Determine which lane(s) are candidate for a lane change, depending on the current lane
	vector<unsigned> candidate_lanes;
	if (car.getLane()==1)
		candidate_lanes= {0, 2};
	else
		candidate_lanes= {1};

	/* Go find the best lane to be, and store its number in fastest_lane
	 *
	 */

	unsigned fastest_lane= car.getLane();  // Will be the best lane
	// Will be the speed of the preceding vehicle in the best lane, or ConfigParams::cruise_speed if there is no preceding vehicle
	double fastest_lane_speed= cars[closest_i].getSpeed();
	bool fastest_lane_empty= false;  // Will be true if there is no car in front of my car in the best lane to be

	// For every lane candidate for a lane change
	for (auto lane: candidate_lanes) {
		// Find the closest preceding and following car in that lane (if any)
		auto preceding= findClosestInLane({car.s,  car.d}, cars, lane, true, ConfigParams::lane_width);
		int preceding_i= preceding.first;
		double preceding_dist= preceding.second;
		auto following= findClosestInLane({car.s,  car.d}, cars, lane, false, ConfigParams::lane_width);
		int following_i= following.first;
		double following_dist= following.second;

		double following_pred_dist= predictSeparation(car, cars[following_i], car.path_x.size()*ConfigParams::tick+ConfigParams::planning_t_CL);

		// If there is no following car, or the following car is at enough distance...
		if (following_i <0 ||
				(abs(following_dist) >= ConfigParams::overtaking_margin_following && abs(following_pred_dist) >=  ConfigParams::overtaking_margin_following)) {
			// if there is no preceding car...
			if (preceding_i < 0) {
					// then mark the lane as the target for lane change
					fastest_lane= lane;
					fastest_lane_speed= ConfigParams::cruise_speed;
					fastest_lane_empty= true;
			}
			// if instead there is a preceding car, but it is distant enought, and haven't found a better lane yet...
			else  if (!fastest_lane_empty &&
					preceding_dist>= ConfigParams::overtaking_margin_preceding &&
					( cars[preceding_i].getSpeed() > fastest_lane_speed || preceding_dist > closest_dist*1.33)) {
				//  then again mark the lane as target for change
				fastest_lane= lane;
				fastest_lane_speed= cars[preceding_i].getSpeed();
				fastest_lane_empty= false;
			}
		}
	}

	// If we have decided for a lane change, then instantiate its corresponding FSM state.
	if (fastest_lane != car.getLane()) {
		auto pNextState= new ChangeLane(car, cars, fastest_lane);
		pNextState->initBoundaryConditions(last_s_boundary_conditions, last_d_boundary_conditions);
		return pNextState;
	}

	return this;
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
	const double prec_car_v= preceding.getSpeed();
	const double time_to_trajectory_end= car.path_x.size()*ConfigParams::tick;
	const double prec_car_s_est= preceding.s+prec_car_v*(time_to_trajectory_end + getPlanningTime());



	// Determine wanted s coordinate for this car at the end of the next planning interval

	double s_p= prec_car_s_est - ConfigParams::safe_distance;  // s pursuit: the s coordinate of the spot I want to reach

	if (s_p < 0)  // Adjust in case s_p is before s=0
		s_p+= ConfigParams::max_s;
	cout << "s_p= " << s_p << endl;

	// If it is requested to go backward, then just decelerate as much as allowed
	if (s_p <= s_start[0]) {
		const double a_max= -ConfigParams::max_accel_s;
		s_goal << s_start[0]+s_start[1]*getPlanningTime()+.5*a_max*pow(getPlanningTime(),2), max(s_start[1]+a_max*getPlanningTime(), 1.), a_max;
		cout << "### Got s_p <=s_start[0] s_p=" << s_p << " s_start[0]=" << s_start[0] << endl;
	}
	else {
		// Acceleration needed to reach s pursuit in getPlanningTime() seconds.
		const double a_to_reach_s_p= 2/pow(getPlanningTime(),2)*(s_p-s_start[0]-s_start[1]*getPlanningTime());
		cout << "a_to_reach_s_p= " << a_to_reach_s_p << endl;
		// Can I afford that acceleration?
		if (abs(a_to_reach_s_p) <= ConfigParams::max_accel_s) {
			// If yes, then just do it!
			s_goal << s_p, prec_car_v, 0;
			cout << "### Can afford max acc a_to_reach_s_p="<< a_to_reach_s_p << endl;
		}
		else {
			// If not, see how far I go with the highest (or lowest) acceleration I can afford, before hitting max speed or stopping
			const double a_max= (a_to_reach_s_p > 0)? ConfigParams::max_accel_s : -ConfigParams::max_accel_s;
			const double v_c= (a_max > 0)? ConfigParams::cruise_speed : .0;
			const double s_c= s_start[0]+(pow(v_c,2)-pow(s_start[1],2))/(2*a_max);
			const double det= max(pow(s_start[1],2)-2*(s_start[0]-s_c)*a_max, .0);
			// How much time to reach s_c?
			cout << "det= " << det << " pow(s_start[1],2)=" << pow(s_start[1],2) << " 2*(s_start[0]-s_c)*a_max=" << 2*(s_start[0]-s_c)*a_max;
			cout << " s_start[1]" << s_start[1] << " s_start[0]" << s_start[0] << endl;
			const double t_c= (-s_start[1]+sqrt(det))/a_max;
			cout << "a_max=" << a_max << " v_c=" << v_c << " s_c=" << s_c << " t_c=" << t_c << endl;
			if (t_c < 0)
				cout << "*** t_c is negative! t_c=" << t_c << endl;
			// Can I make it to s_c by the end of the planning time interval?
			if (t_c >  getPlanningTime()) {
				// If not, just keep accelerating until the end of the planning time interval
				s_goal << s_start[0]+s_start[1]*getPlanningTime()+.5*a_max*pow(getPlanningTime(),2), s_start[1]+a_max*getPlanningTime(), a_max;
				cout << "### Cannot make it to s_c t_c=" << t_c << " getPlanningTime()=" << getPlanningTime() << endl;
			}
			else {
				// If yes, calculate until where I carry on proceeding at v_c speed after I reach s_c
				const double s_final= s_c+v_c * (getPlanningTime()-t_c);
				cout << "s_final=" << s_final << endl;
				// Would I make it past s_p?
				if (s_final >= s_p) {
					// Then just plan to reach s_p
					s_goal << s_p, prec_car_v, 0;
					cout << "### Cannot reach s_p at v_c speed s_final=" << s_final << " s_p="<< s_p << endl;
				}
				else {
					// Otherwise plan to go on at constant speed
					s_goal << s_final, max(v_c, 1.), 0;
					cout << "### Just continuing at constant speed s_final=" << s_final << " s_c=" << s_c << endl;
				}
			}
		}
	}

	cout << "FollowCar s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

	Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
	Vector3d d_goal;// Goal conditions for d
	d_goal << d_forLane(car.getLane()), 0, 0;
	cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

	return { s_goal, d_goal };
}

KeepLane::KeepLane(const Car & car_init, const std::vector<CarSensorData> cars_init):
		FSM_State(car_init, cars_init) {
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


pair<Vector3d, Vector3d> KeepLane::computeGoalBoundaryConditions() {
	assert(boundary_conditions_initialised);
	Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
	Vector3d s_goal;// Goal conditions for s

	double target_speed_s= ConfigParams::cruise_speed;

	int a_sign = (target_speed_s > s_start[1])? 1 : -1;
	double proj_vel_s = s_start[1] + a_sign*ConfigParams::max_accel_s*ConfigParams::planning_t_KL;
	if ((a_sign > 0 && proj_vel_s < target_speed_s) || (a_sign < 0 && proj_vel_s > target_speed_s))
		s_goal << s_start[0]+s_start[1]*getPlanningTime()+.5*(a_sign)*ConfigParams::max_accel_s*pow(getPlanningTime(),2), proj_vel_s, a_sign*ConfigParams::max_accel_s;
	else {
		double tx= a_sign*(target_speed_s - s_start[1]) / ConfigParams::max_accel_s;
		double s1= s_start[0] + s_start[1]*tx+.5*a_sign*ConfigParams::max_accel_s*pow(tx,2);
		double s2= (getPlanningTime() - tx) * target_speed_s;
		s_goal << s1+s2, target_speed_s, 0;
	}
	cout << "KeepLane s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

	Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
	Vector3d d_goal;// Goal conditions for d
	d_goal << d_forLane(car.getLane()), 0, 0;
	cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

	return { s_goal, d_goal };
}


ChangeLane::ChangeLane(const Car & car_init, const std::vector<CarSensorData> cars_init, const unsigned target_lane_init):
		FSM_State(car_init, cars_init), target_lane(target_lane_init) {
	cout << "Instantiated ChangeLane with target_lane= "<< target_lane_init << endl;
}

FSM_State * ChangeLane::getNextState(const Car & theCar, const std::vector<CarSensorData> theCars) {
	car= theCar;
	cars= theCars;

	// Is the change of lane complete?
	if (abs(car.d-d_forLane(target_lane)) < .2)  {
		auto pNextState= new KeepLane(car, cars);
		pNextState->initBoundaryConditions(last_s_boundary_conditions, last_d_boundary_conditions);
		return pNextState;
	}
	return this;
}

pair<Vector3d, Vector3d> ChangeLane::computeGoalBoundaryConditions() {
	Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
	Vector3d s_goal;// Goal conditions for s

	s_goal << s_start[0]+s_start[1]*getPlanningTime(), s_start[1], 0;
	cout << "ChangeLane s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

	Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
	Vector3d d_goal;// Goal conditions for d
	d_goal << d_forLane(target_lane), 0, 0;
	cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

	// Update the boundary conditions to be used at the beginning of the next JMT
	/*last_s_boundary_conditions = s_goal;
	if (last_s_boundary_conditions[0] >= ConfigParams::max_s)
		last_s_boundary_conditions[0]-= ConfigParams::max_s;
	last_d_boundary_conditions = d_goal; */

	return {s_goal, d_goal};
}



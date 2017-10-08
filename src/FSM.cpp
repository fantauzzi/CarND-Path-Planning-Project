#include <climits>
#include <utility>
#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "car.h"
#include "FSM.h"

using namespace std;
using namespace Eigen;

pair<double, double> findClosestInLane(Coordinates sd,  vector<carSensorData> cars, unsigned lane, bool preceding, double lane_width ) {
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

Vector6d computeJMT(const Vector3d start, const Vector3d goal, double t) {
	auto a0 = start[0];
	auto a1 = start[1];
	auto a2 = start[2] / 2.;

	Matrix3d A;
	A << pow(t, 3), pow(t, 4), pow(t, 5), 3 * pow(t, 2), 4 * pow(t, 3), 5
			* pow(t, 4), 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

	auto c0 = a0 + a1 * t + a2 * pow(t, 2);
	auto c1 = a1 + 2 * a2 * t;
	auto c2 = 2 * a2;

	Vector3d b;
	b << goal[0] - c0, goal[1] - c1, goal[2] - c2;

	Vector3d x = A.colPivHouseholderQr().solve(b);
	Vector6d result;
	result << a0, a1, a2, x;
	return result;
}
FSM_State::FSM_State(
		double car_s_init,
		double car_d_init,
		unsigned lane_init,
		double cruise_speed_init,
		double lane_width_init,
		double max_accel_s_init,
		double planning_t_init,
		double max_s_init):
				car_s(car_s_init),
				car_d(car_d_init),
				lane(lane_init),
				cruise_speed(cruise_speed_init),
				lane_width(lane_width_init),
				max_accel_s(max_accel_s_init),
				planning_t(planning_t_init),
				max_s(max_s_init){

}

FSM_State::~FSM_State() {
	// do nothing
}


KeepLane::KeepLane(
		double car_s,
		double car_d,
		unsigned lane,
		double cruise_speed,
		double lane_width,
		double max_accel_s,
		double planning_t,
		double max_s): FSM_State(car_s,
				car_d,
				lane,
				cruise_speed,
				lane_width,
				max_accel_s,
				planning_t,
				max_s) {

}

FSM_State * KeepLane::getNextState() {
	// Find the closest vehicle in range preceding in the same lane (if any)
	auto closest_info= findClosestInLane({car_s, car_d}, cars, lane, true, lane_width);
	int closest_i= closest_info.first;  // Will be the position in cars[] of the found vehicle (if found)
	double closest_dist=  closest_info.second;

	/* If the distance is below a certain amount, set the target speed to the speed
	 * of the preceding car, less some margin; otherwise set the target speed to the max cruise speed
	 */
	bool consider_lane_change= false;
	if (closest_i >= 0 && closest_dist <= 50) {
		target_speed= min(sqrt(pow(cars[closest_i].vx,2)+pow(cars[closest_i].vy,2))*.99, cruise_speed);
		consider_lane_change= true;  // TODO not quite right, what if the preceding car is faster than cruise_speed?
		// cout << "Separation=" << closest_dist << " car#" <<closest_i << " target_speed=" << target_speed << endl;
	}
	else
		target_speed= cruise_speed;

	if (consider_lane_change) {
		cout << "Considering lane change." << endl;
		// Which lanes should we consider for a lane change?
		vector<unsigned> lanes;
		if (lane==1)
			lanes= {0u, 2u};
		else
			lanes= {1u};
		int new_lane= -1;  // Will be set to the best lane to change to, or left set to -1 if no suitable lane change is found
		double new_lane_speed= 0;

		for (auto the_lane: lanes) {
			// Find closest preceding vehicle (if any) in adjacent lane
			auto preceding = findClosestInLane({car_s, car_d }, cars, the_lane, true, lane_width);
			int preceding_i= preceding.first;
			double preceding_dist= preceding.second;
			if (preceding_i >=0)
				cout << "Found preceding car in lane " << the_lane << " with distance " << preceding_dist << endl;
			auto following = findClosestInLane({car_s, car_d }, cars, the_lane, false, lane_width);
			int following_i= following.first;
			double following_dist= following.second;
			if (following_i >=0)
				cout << "Found following car in lane " << the_lane << " with distance " << following_dist << endl;
			// Is this lane change viable?
			bool viable= true;
			if (following_i >=0 && following_dist < 10)
				viable= false;
			else if (preceding_i >=0 && preceding_dist <15)
				viable= false;
			if (viable && (preceding_i<0 || (preceding_i>=0 && cars[preceding_i].getSpeed() > new_lane_speed))) {
				new_lane= the_lane;
				new_lane_speed= (preceding_i >= 0)? cars[preceding_i].getSpeed() : cruise_speed;
			}

		}
	}
	return this;
}

void KeepLane::computeBoundaryConditions() {
	Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
	Vector3d s_goal;// Goal conditions for s
	int a_sign = (target_speed > s_start[1])? 1 : -1;
	double proj_vel_s = s_start[1] + a_sign*max_accel_s*planning_t;
	if ((a_sign > 0 && proj_vel_s < target_speed) || (a_sign < 0 && proj_vel_s > target_speed))
		s_goal << s_start[0]+s_start[1]*planning_t+.5*(a_sign)*max_accel_s*pow(planning_t,2), proj_vel_s, a_sign*max_accel_s;
	else {
		double tx= a_sign*(target_speed - s_start[1]) / max_accel_s;
		double s1= s_start[0] + s_start[1]*tx+.5*a_sign*max_accel_s*pow(tx,2);
		double s2= (planning_t - tx) * target_speed;
		s_goal << s1+s2, target_speed, 0;
	}
	cout << "s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

	Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
	Vector3d d_goal;// Goal conditions for d
	d_goal << 2+lane*4, 0, 0;
	cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

	// Update the boundary conditions to be used at the beginning of the next JMT
	last_s_boundary_conditions = s_goal;
	if (last_s_boundary_conditions[0] >= max_s)
		last_s_boundary_conditions[0]-=max_s;
	last_d_boundary_conditions = d_goal;

	// Compute the quintic polynomial coefficients, for the given boundary conditions and planning time interval
	auto sJMT = computeJMT(s_start, s_goal, planning_t);
	cout << "sJMT= " << sJMT.transpose() << endl << endl;
	auto dJMT = computeJMT(d_start, d_goal, planning_t);
	cout << "dJMT= " << dJMT.transpose() << endl << endl;
}

KeepLane::~KeepLane() {
	// do nothing
}


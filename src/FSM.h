#pragma once
#include <vector>
#include <utility>
#include "Eigen/Core"

class FSM_State {
protected:
	std::vector<carSensorData> cars;
	double car_s;  // TODO group all this stuff in its own class/struct
	double car_d;
	unsigned lane;
	double cruise_speed;
	double target_speed;  // TODO should it be here or in a derived class
	double lane_width;
	Eigen::Vector3d last_s_boundary_conditions;
	Eigen::Vector3d last_d_boundary_conditions;
	double max_accel_s;
	double planning_t;
	double max_s;
public:
	FSM_State(double car_s, double car_d, unsigned lane, double cruise_speed, double lane_width, double max_accel_s, double planning_t, double max_s);
	virtual FSM_State * getNextState()=0;
	virtual void computeBoundaryConditions()=0;
	virtual ~FSM_State();
};

class KeepLane: public FSM_State {
public:
	KeepLane(double car_s, double car_d, unsigned lane, double cruise_speed, double lane_width, double max_accel_s, double planning_t, double max_s);
	virtual FSM_State * getNextState() override;  // TODO could I narrow to return a KeepLane * ?
	virtual void computeBoundaryConditions() override;
	virtual ~KeepLane();
};

std::pair<double, double> findClosestInLane(Coordinates sd,  std::vector<carSensorData> cars, unsigned lane, bool preceding, double lane_width );

/* Convenient alias for a type, will hold the 6 coefficients of a quintic function, sorted
 * from the degree 0 coefficient to the degree 5: a0+a1*x+a2*x^2+a3*x^3+a4*x^4+a5*x^5 .
 */
typedef Eigen::Matrix<double, 6, 1> Vector6d;


/**
 * Computes the jerk minimising trajectory for the given boundary conditions and time interval.
 * @param start the starting boundary conditions.
 * @param goal the ending boundary conditions.
 * @param t the time interval in seconds to reach the goal from start.
 * @return the coefficients of a quintic function, providing the requested trajectory, ordered
 * from the term of degree 0 to the term of degree 5.
 */
Vector6d computeJMT(const Eigen::Vector3d start, const Eigen::Vector3d goal, double t);

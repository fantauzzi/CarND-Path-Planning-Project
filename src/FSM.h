#pragma once
#include <vector>
#include <utility>
#include "Eigen/Core"

#include "Car.h"

class FSM_State {
protected:
	Car car;
	std::vector<CarSensorData> cars;
	Eigen::Vector3d last_s_boundary_conditions;
	Eigen::Vector3d last_d_boundary_conditions;
public:
	FSM_State(const Car car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState()=0;
	virtual void computeBoundaryConditions()=0;
	virtual ~FSM_State();
};

class KeepLane: public FSM_State {
	double target_speed;
public:
	KeepLane(const Car & car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState() override;  // TODO could I narrow to return a KeepLane * ?
	virtual void computeBoundaryConditions() override;
	virtual ~KeepLane();
};

std::pair<double, double> findClosestInLane(Coordinates sd,  std::vector<CarSensorData> cars, unsigned lane, bool preceding, double lane_width );




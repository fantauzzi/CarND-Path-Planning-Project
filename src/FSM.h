#pragma once
#include <vector>
#include <utility>
#include "Eigen/Core"
#include "trajectory.h"
#include "Car.h"

class FSM_State {
protected:
	Car car;
	std::vector<CarSensorData> cars;
	Eigen::Vector3d last_s_boundary_conditions;
	Eigen::Vector3d last_d_boundary_conditions;
	bool boundary_conditions_initialised;
public:
	FSM_State(const Car car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars)=0;
	virtual std::pair<Vector6d, Vector6d> computeBoundaryConditions()=0;
	virtual void initBoundaryConditions(Eigen::Vector3d s_init, Eigen::Vector3d d_init);
	virtual ~FSM_State() {};
};

class KeepLane: public FSM_State {
	double target_speed;
public:
	KeepLane(const Car & car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars) override;  // TODO could I narrow to return a KeepLane * ?
	virtual std::pair<Vector6d, Vector6d> computeBoundaryConditions() override;
	virtual ~KeepLane() {};
};

std::pair<double, double> findClosestInLane(Coordinates sd,  std::vector<CarSensorData> cars, unsigned lane, bool preceding, double lane_width );




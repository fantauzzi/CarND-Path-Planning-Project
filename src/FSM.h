#pragma once
#include <vector>
#include <utility>
#include "Eigen/Core"
#include "trajectory.h"
#include "Car.h"
#include "ConfigParams.h"

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
	virtual double getPlanningTime() const =0;
	virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> getGoalConditions(const double target_s,
			const double target_d,
			const double target_v,
			const double planning_t) const;
	virtual ~FSM_State() {};
};

class KeepLane: public FSM_State {
	double target_speed;  // TODO do I need it as a data member?
public:
	KeepLane(const Car & car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars) override;
	virtual std::pair<Vector6d, Vector6d> computeBoundaryConditions() override;
	virtual double getPlanningTime() const override {
		return ConfigParams::planning_t_KL;
	}
	virtual ~KeepLane() {};
};

class FollowCar: public FSM_State {
	double target_speed;  // TODO do I need it as a data member?
public:
	FollowCar(const Car & car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars) override;
	virtual std::pair<Vector6d, Vector6d> computeBoundaryConditions() override;
	virtual double getPlanningTime() const override {
		return ConfigParams::planning_t_KL;
	}
	virtual ~FollowCar() {};
};

class ChangeLane: public FSM_State {
	unsigned target_lane;
	double target_speed;
public:
	ChangeLane(const Car & car, const std::vector<CarSensorData> cars, const unsigned target_lane, const double target_speed);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars) override;  // TODO could I narrow to return a KeepLane * ?
	virtual std::pair<Vector6d, Vector6d> computeBoundaryConditions() override;
	virtual double getPlanningTime() const override {
		return ConfigParams::planning_t_CL;
	}
	virtual ~ChangeLane() {};
};


std::pair<double, double> findClosestInLane(Coordinates sd,  std::vector<CarSensorData> cars, unsigned lane, bool preceding, double lane_width );




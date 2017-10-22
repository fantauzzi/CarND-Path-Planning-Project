#pragma once
#include <vector>
#include <utility>
#include "Eigen-3-3-4/Eigen/Core"
#include "trajectory.h"
#include "Car.h"
#include "ConfigParams.h"

/**
 * Checks if two numbers are close enough to be considered the same.
 * @param a one of the two given numbers.
 * @param b the other given number.
 * @return true if the difference in absolute value between the two numbers is within tolerance of 0.001, false otherwise.
 */
bool close_enough(const double a, const double b);

/**
 * Finds in a given lane the closest preceding or following vehicle (if any) to the given Frenet coordinates.
 * @param sd Frenet coordinates as a pair <s, d>.
 * @param cars sensors provided information about the other vehicles on the same road.
 * @param lane the lane of interest.
 * @param preceding true if the closest preceding vehicle is wanted, false if the closes following vehicle is wanted.
 * @param lane_width the lane width in meters.
 * @return a pair <index, distance> where `index` is the position in the given `cars` vector of the found vehicle, -1 if no
 * vehicle was found at all, and `distance` is the distance with sign (positive for a preceding car) of the found vehicle,
 * numeric_limits<double>::max() if none was found.
 */
std::pair<double, double> findClosestInLane(Coordinates sd,  std::vector<CarSensorData> cars, unsigned lane, bool preceding, double lane_width );

/**
 * Base class for all the FMS states. They are responsible to determine the next state for the vehicle,
 * compute the goal boundary conditions for the vehicle trajectory, and generate the trajectory.
 */
class FSM_State {
protected:
	Car car;  // The vehicle for which we are tracking the state
	std::vector<CarSensorData> cars;

	/* Goal boundary conditions for s and d at the end of the previous iteration, to be
	 * used as starting boundary conditions for the current iteration.
	 */
	Eigen::Vector3d last_s_boundary_conditions;
	Eigen::Vector3d last_d_boundary_conditions;

	/* true iff the previous two data members have been initialised already (i.e. we are past the first iteration)
	 *
	 */
	bool boundary_conditions_initialised;

public:
	/**
	 * Constructs the object initialising it with the given parameters; note that
	 * last_s_boundary_conditions, last_d_boundary_conditions and boundary_conditions_initialised
	 * must be initialised separately by calling initBoundaryConditions().
	 * @param information about the vehicle for which we want to track the state.
	 * @param cars sensors information about other vehicles.
	 */
	FSM_State(const Car car, const std::vector<CarSensorData> cars);

	/**
	 * Fetches the next vehicle state.
	 * @param information about the vehicle for which we want to track the state.
	 * @param cars sensors information about other vehicles.
	 * @return a pointer to the instance of FSM_State giving the updated vehicle state;
	 * if the state is the same as before, then the pointer is `this`. The caller must
	 * take ownership of the pointer.
	 */
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars)=0;

	/**
	 * Given the current state, computes the goal boundary conditions for the next vehicle
	 * trajectory. initBoundaryConditions() must be called once first.
	 * @return a pair <s_goal, d_goal> with the goal boundary conditions for, respectively, the
	 * s and d coordinate.
	 */
	virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> computeGoalBoundaryConditions()=0;

	/**
	 * Call during the first iteration, after setting the first path in the simulation, to record
	 * its goal boundary conditions in last_s_boundary_conditions and last_d_boundary_conditions;
	 * also boundary_conditions_initialised is set to true;
	 * @param s_init the goal boundary conditions for s
	 * @param d_init the goal boundary conditions for d
	 */
	virtual void initBoundaryConditions(Eigen::Vector3d s_init, Eigen::Vector3d d_init);

	/**
	 * Gets the total duration of the planned trajectory.
	 * @return the duration in seconds.
	 */
	virtual double getPlanningTime() const =0;

	/**
	 * Generates a trajectory given the current car state.
	 * @return a pair <s_t, d_t>, the coefficients of the quintic polynomials
	 * describing respectively the trajectory s(t) and d(t). Coefficients
	 * are ordered starting with the one of degree 0.
	 */
	virtual std::pair<Vector6d, Vector6d> generateTrajectory();

	/**
	 * Virtual destructor that does nothing.
	 */
	virtual ~FSM_State() {};
};

/**
 * In this state, the car keeps the current lane and tries to attain and maintain its
 * cruise speed, as set in ConfigParams::cruise_speed.
 */
class KeepLane: public FSM_State {
public:
	KeepLane(const Car & car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars) override;
	virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> computeGoalBoundaryConditions() override;
	virtual double getPlanningTime() const override {
		return ConfigParams::planning_t_KL;
	}
	virtual ~KeepLane() {};
};

/**
 * In this state, the car tries to keep a set distance from a preceding car in the same lane;
 * the distance is set in ConfigParams::safe_distance.
 */
class FollowCar: public FSM_State {
public:
	FollowCar(const Car & car, const std::vector<CarSensorData> cars);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars) override;
	virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> computeGoalBoundaryConditions() override;
	virtual double getPlanningTime() const override {
		return ConfigParams::planning_t_KL;
	}
	virtual ~FollowCar() {};
};

/**
 * In this state the car changes lane; the new lane is passed to the constructor and stored in `target_lane`.
 */
class ChangeLane: public FSM_State {
	unsigned target_lane;
public:
	ChangeLane(const Car & car, const std::vector<CarSensorData> cars, const unsigned target_lane);
	virtual FSM_State * getNextState(const Car & car, const std::vector<CarSensorData> cars) override;
	virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> computeGoalBoundaryConditions() override;
	virtual double getPlanningTime() const override {
		return ConfigParams::planning_t_CL;
	}
	virtual ~ChangeLane() {};
};

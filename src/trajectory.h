#pragma once
#include "Eigen/Core"


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


double evalQuintic(Vector6d coeffs, double x);

double measureQuinticArcLength(const Vector6d coeffs, const double x0, const double x1, const double step);

/**
 * Calculates the jerk minimising trajectory for the given car to get, or remain, in the given lane.
 * @param car the given car.
 * @param target_lane the number of the lane where the car should end up
 * @param time the planned duration, in seconds, for the trajectory
 * @return the coefficients of a quintic polynomial providing the desired trajectory
 */
// Vector6d computeJMT(const Car & car, const unsigned target_lane, const double time);



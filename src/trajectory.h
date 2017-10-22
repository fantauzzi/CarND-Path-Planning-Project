#pragma once
#include "Eigen-3-3-4/Eigen/Core"

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

/**
 * Evaluates a given quintic polynomial for the given value
 * @param coeffs coefficients of the quintic polynomial, ordered starting from the one of degree 0.
 * @param x the given point, where to evaluate the polynomial.
 * @return the quintic polynomial value in `x`
 */
double evalQuintic(Vector6d coeffs, double x);


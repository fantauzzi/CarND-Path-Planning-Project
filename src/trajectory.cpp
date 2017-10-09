#include "Eigen/QR"
#include <cmath>
#include "trajectory.h"

using namespace Eigen;

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


/**
 * Evaluates the quintic function with the given coefficients in `x`.
 * @param coeffs the quintic function coefficients, ordered from the term of degree 0 to the term of degree 5.
 * @param x the value for which to evaluate the quintic function.
 * @return the computed value.
 */
double evalQuintic(Vector6d coeffs, double x) {
	double result = coeffs[0] + coeffs[1] * x + coeffs[2] * pow(x, 2)
			+ coeffs[3] * pow(x, 3) + coeffs[4] * pow(x, 4)
			+ coeffs[5] * pow(x, 5);
	return result;
}

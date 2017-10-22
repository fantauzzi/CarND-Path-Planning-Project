#pragma once
#include <vector>
#include <utility>
#include "spline.h"

// Short-hand for convenience
typedef std::pair<double, double> Coordinates;

/**
 * @return the Pi constant.
 */
constexpr double pi() {
	return M_PI;
}

/**
 * Converts from degrees to radians.
 */
double deg2rad(double x);

/**
 * Converts from radians to degrees.
 */
double rad2deg(double x);

/**
 * Computes the distance between two points in Cartesian coordinates. It is always non-negative.
 */
double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
		const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
		const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/**
 * The class stores map information and uses it to convert between Frenet and Cartesian coordinates.
 */
class FrenetCartesianConverter {
	std::vector<double> maps_s;
	std::vector<double> maps_x;
	std::vector<double> maps_y;
	std::vector<double> maps_dx;
	std::vector<double> maps_dy;

	tk::spline spline_maps_x;
	tk::spline spline_maps_y;
	tk::spline spline_maps_dx;
	tk::spline spline_maps_dy;

public:
	FrenetCartesianConverter(const std::vector<double> maps_s,
			const std::vector<double> maps_x, const std::vector<double> maps_y,
			const std::vector<double> map_waypoints_dx,
			const std::vector<double> map_waypoints_dy);

	std::pair<double, double> getXY(const double s, const double d) const;

	std::pair<double, double> getFrenet(const double x, const double y,
			const double theta) const;

	/**
	 * Obtains the road heading at a given position, in radians. The heading is an angle between 0
	 * and two times Pi, with 0 along the positive direction of the x-axis (due-East), and
	 * increasing counter-clockwise.
	 * @param s the s coordinate where the road heading is wanted.
	 * @return the calculated road heading.
	 */
	double getRoadHeading(const double s) const;
};

/**
 * Converts Cartesian coordinates from the universal reference system to the car reference system.
 */
Coordinates universal2car_ref(const Coordinates,
		const double car_x, const double car_y, const double car_yaw);

/**
 * Converts Cartesian coordinates from the car reference system to the universal reference system.
 */
Coordinates car2universal_ref(const Coordinates,
		const double car_x, const double car_y, const double car_yaw);

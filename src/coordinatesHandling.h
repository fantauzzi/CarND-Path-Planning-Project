#pragma once
#include <vector>
#include <utility>
#include "spline.h"

typedef std::pair<double, double> Coordinates;

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
		const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
		const std::vector<double> &maps_x, const std::vector<double> &maps_y);

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

	double getRoadHeading(const double s) const;
};


Coordinates universal2car_ref(const Coordinates,
		const double car_x, const double car_y, const double car_yaw);

Coordinates car2universal_ref(const Coordinates,
		const double car_x, const double car_y, const double car_yaw);

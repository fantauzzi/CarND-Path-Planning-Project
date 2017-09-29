#pragma once
#include <vector>
#include <utility>
#include "spline.h"

// For converting back and forth between radians and degrees.
inline constexpr double pi();

inline double deg2rad(double x);

inline double rad2deg(double x);

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
			const std::vector<double> maps_x, const std::vector<double> maps_y, const std::vector<double> map_waypoints_dx, const std::vector<double> map_waypoints_dy);

	std::pair<double, double> getXY(const double s, const double d) const;

	std::pair<double, double> getXY2(const double s, const double d) const;

	std::pair<double, double> getFrenet(const double x, const double y,
			const double theta) const;
};

#pragma once
#include <vector>
#include <utility>

// For converting back and forth between radians and degrees.
inline constexpr double pi();

inline double deg2rad(double x);

inline double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
		const std::vector<double> &maps_y);


int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x,
		const std::vector<double> &maps_y);

class FrenetCartesianConverter {
	std::vector<double> maps_s;
	std::vector<double> maps_x;
	std::vector<double> maps_y;

public:
	FrenetCartesianConverter(const std::vector<double> maps_s,
			const std::vector<double> maps_x, const std::vector<double> maps_y);
	std::pair<double, double> getXY(const double s, const double d) const;
	std::pair<double, double> getFrenet(const double x, const double y, const double theta) const;
};



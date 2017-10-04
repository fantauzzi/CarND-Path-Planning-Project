#pragma once
#include <utility>
#include <vector>
#include "coordinatesHandling.h"

struct carSensorData {
	unsigned id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	FrenetCartesianConverter converter;
	carSensorData(std::vector<double> sensorInfo, const FrenetCartesianConverter & the_converter);
	double measureDistanceFrom(const double x, const double y) const;
	Coordinates getFrenetVelocity() const;
	double getSpeed() const;
	double getYaw() const;
	Coordinates predictFrenet(double dt) const;
	double measure_sSeparationFrom(const double s) const;
};

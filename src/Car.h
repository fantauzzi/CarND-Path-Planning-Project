#pragma once
#include <utility>
#include <vector>
#include "coordinatesHandling.h"

struct CarSensorData {
	unsigned id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	FrenetCartesianConverter converter;
	CarSensorData(std::vector<double> sensorInfo, const FrenetCartesianConverter & the_converter);

	/**
	 * Measure the Euclidean car distance from the given point in a Cartesian refence system.
	 * @param from_x the x coordinate of the point to take the distance from.
	 * @param from_y the y coordinate of the point to take the distance from.
	 * @return the measured distance.
	 */
	double measureDistanceFrom(const double x, const double y) const;

	/**
	 * Compute and return the car velocity components along the s and d axis of a
	 * Frenet reference system.
	 * @return a pair whose two elements are respectively the velocity along s and along d.
	 */
	Coordinates getFrenetVelocity() const;

	/**
	 * Compute and return the velocity value (modulus) in m/s of the car.
	 * @return the computed velocity.
	 */
	double getSpeed() const;

	/**
	 * Compute and return the car yaw in radians. It is an angle taken counter-clockwise from
	 * the x axis, ranging from 0 to 2 times Pi, with 0 corresponding to the direction of the
	 * positive x axis.
	 * @return the computed yaw angle.
	 */
	double getYaw() const;

	/**
	 * Compute and return the car predicted position in Frenet coordinates in a given time interval.
	 * @param dt the time interval, expressed in seconds.
	 * @return a pair consisting in the prediceted s and d coordinates, respectively.
	 */
	Coordinates predictFrenet(double dt) const;

	/**
	 * Compute and return the distance with sign between a point at the given s Frenet coordinate
	 * and this car; the distance is positive if the point is in front of the car, negative otherwise.
	 * @param other_s the s coordinate of the point from which to take the distance.
	 * @return the computed distance with sign.
	 */
	double measureSeparationFrom(const double s) const;
};


struct Car {
	double s;
	double d;
	double x;
	double y;
	double yaw;
	double speed;
	std::vector<double> path_x;
	std::vector<double> path_y;
	unsigned getLane() const;
};

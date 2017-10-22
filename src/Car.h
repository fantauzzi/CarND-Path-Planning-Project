#pragma once
#include <utility>
#include <vector>
#include "coordinatesHandling.h"

/**
 * Measure the distance with sign between two vehicles with given `s` Frenet coordinate. The function handles
 * correctly the case where the two cars are on opposite sides of the s=0 line (i.e. where the track wraps around)
 * @param this_car_s s coordinate of the first car.
 * @param other_car_s s coordinate of the second car.
 * @return the distance bewteen the two cars along the s axis, with positive sign if the second car precedes the first, negative otherwise.
 */
double measureSeparation(const double this_car_s, const double other_car_s);

/**
 * Information provided by the sensors (simulator) about every other vehicle in range.
 * Measurement units are all I.S..
 */
struct CarSensorData {
	unsigned id;  // An ID unique to the given vehicle
	double x;
	double y;
	double vx;  // Velocity along the x axis
	double vy;  // Velocity along the y axis
	double s;
	double d;
	FrenetCartesianConverter converter;  // An object in charge for coordinates conversion

	/**
	 * Instantiates a CarSensorData object.
	 * @param sensorInfo a vector to initialise the sensor information, holding in this order: car id, x coordinate,
	 * t coordinate, x velocity, y velocity, s coordinate, d coordinate,
	 * @param the_converter reference to the object that will be in charge for conversion, the constructor take a
	 * copy of it.
	 */
	CarSensorData(const std::vector<double> sensorInfo, const FrenetCartesianConverter & the_converter);

	/**
	 * Measures the Euclidean car distance from the given point in a Cartesian refence system.
	 * @param from_x the x coordinate of the point to take the distance from.
	 * @param from_y the y coordinate of the point to take the distance from.
	 * @return the measured distance.
	 */
	double measureDistanceFrom(const double x, const double y) const;

	/**
	 * Computes and return the velocity value (modulus) in m/s of the car.
	 * @return the computed velocity.
	 */
	double getSpeed() const;

	/**
	 * Computes and return the car yaw in radians. It is an angle taken counter-clockwise from
	 * the x axis, ranging from 0 to 2 times Pi, with 0 corresponding to the direction of the
	 * positive x axis.
	 * @return the computed yaw angle.
	 */
	double getYaw() const;

	/**
	 * Computes and return the car predicted position in Frenet coordinates in a given time interval.
	 * @param dt the time interval, expressed in seconds.
	 * @return a pair consisting in the prediceted s and d coordinates, respectively.
	 */
	Coordinates predictFrenet(double dt) const;
};

/**
 *  Information related to the car being driven. All measurement units are I.S..
 */
struct Car {
	double s;
	double d;
	double x;
	double y;
	double yaw;  // In radians, 0 being the positive direction of the x axis ("due East").
	double speed;  // Yes, in m/s
	std::vector<double> path_x;  // X coordinate of waypoints not yet driven, path_x[0] is the one the most far away from the car
	std::vector<double> path_y;  // Y coordinate of waypoints not yet driven
	FrenetCartesianConverter converter;

	/**
	 * Instantiate a Car object, with an associated FrenetCartesianConverter.
	 * @param converter the object in charge for coordinates conversion.
	 */
	Car(const FrenetCartesianConverter & converter);
	unsigned getLane() const;
};

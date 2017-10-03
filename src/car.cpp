#include "car.h"
#include <cmath>
#include <utility>
#include "coordinatesHandling.h"

using namespace std;

carSensorData::carSensorData(vector<double> sensorInfo, const FrenetCartesianConverter & the_converter):
		id(static_cast<unsigned>(sensorInfo[0])),
		x(sensorInfo[1]),
		y(sensorInfo[2]),
		vx(sensorInfo[3]),
		vy(sensorInfo[4]),
		s(sensorInfo[5]),
		d(sensorInfo[6]),
		converter(the_converter)
{
	assert(sensorInfo.size()==7);
}

/**
 * Measure the Euclidean car distance from the given point in a Cartesian refence system.
 * @param from_x the x coordinate of the point to take the distance from.
 * @param from_y the y coordinate of the point to take the distance from.
 * @return the measured distance.
 */
double carSensorData::measureDistanceFrom(const double from_x, const double from_y) const {
	return sqrt(pow(x-from_x, 2)+pow(y-from_y, 2));
}

/**
 * Compute and return the velocity value (modulus) in m/s of the car.
 * @return the computed velocity.
 */
double carSensorData::getSpeed() const {
	return sqrt(vx*vx+vy*vy);
}

/**
 * Compute and return the car yaw in radians. It is an angle taken counter-clockwise from
 * the x axis, ranging from 0 to 2 times Pi, with 0 corresponding to the direction of the
 * positive x axis.
 * @return the computed yaw angle.
 */
double carSensorData::getYaw() const {
	double yaw= atan2(vy, vx);
	// Make sure the angle is between 0 and pi radians
	if (yaw<0)
		yaw += 2*pi();
	return yaw;
}

/**
 * Compute and return the car velocity components along the s and d axis of a
 * Frenet reference system.
 * @return a pair whose two elements are respectively the velocity along s and along d.
 */
Coordinates carSensorData::getFrenetVelocity() const {
	double road_h = converter.getRoadHeading(s);
	double car_vel_s= getSpeed() *cos(getYaw() - road_h);
	double car_vel_d= -getSpeed()*sin(getYaw() - road_h); // d=0 on the yellow center line, and increases toward the outer of the track
	return { car_vel_s, car_vel_d };
}

/**
 * Compute and return the car predicted position in Frenet coordinates in a given time interval.
 * @param dt the time interval, expressed in seconds.
 * @return a pair consisting in the prediceted s and d coordinates, respectively.
 */
Coordinates carSensorData::predictFrenet(double dt) const {
	const auto sd_vel = getFrenetVelocity();
	const double s_pred = s+sd_vel.first * dt;
	const double d_pred = d+sd_vel.second * dt;
	return { s_pred, d_pred };
}

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


double carSensorData::measureDistanceFrom(const double from_x, const double from_y) const {
	return sqrt(pow(x-from_x, 2)+pow(y-from_y, 2));
}


double carSensorData::getSpeed() const {
	return sqrt(vx*vx+vy*vy);
}


double carSensorData::getYaw() const {
	double yaw= atan2(vy, vx);
	// Make sure the angle is between 0 and pi radians
	if (yaw<0)
		yaw += 2*pi();
	return yaw;
}


Coordinates carSensorData::getFrenetVelocity() const {
	double road_h = converter.getRoadHeading(s);
	double car_vel_s= getSpeed() *cos(getYaw() - road_h);
	double car_vel_d= -getSpeed()*sin(getYaw() - road_h); // d=0 on the yellow center line, and increases toward the outer of the track
	return { car_vel_s, car_vel_d };
}


Coordinates carSensorData::predictFrenet(double dt) const {
	const auto sd_vel = getFrenetVelocity();
	const double s_pred = s+sd_vel.first * dt;
	const double d_pred = d+sd_vel.second * dt;
	return { s_pred, d_pred };
}


double carSensorData::measureSeparationFrom(const double other_s) const {
	double sep= other_s -s;

	// Handle the case where this car are the other car are on opposite side of the line s=0
	if (sep < -converter.max_s/2 || sep > converter.max_s/2)
		sep = converter.max_s-sep;

	return sep;
}

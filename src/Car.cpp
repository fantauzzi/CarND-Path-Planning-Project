#include <cmath>
#include <utility>
#include "coordinatesHandling.h"
#include "Car.h"
#include "ConfigParams.h"

using namespace std;

double measureSeparation(const double this_car_s, const double other_car_s) {
	double sep= other_car_s -this_car_s;

	// Handle the case where this car are the other car are on opposite side of the line s=0
	if (sep < -ConfigParams::max_s/2 )
		sep = other_car_s+ConfigParams::max_s-this_car_s;
	else if (sep > ConfigParams::max_s/2)
		sep = this_car_s+ConfigParams::max_s-other_car_s;

	return sep;
}

CarSensorData::CarSensorData(const vector<double> sensorInfo, const FrenetCartesianConverter & the_converter):
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


double CarSensorData::measureDistanceFrom(const double from_x, const double from_y) const {
	return sqrt(pow(x-from_x, 2)+pow(y-from_y, 2));
}


double CarSensorData::getSpeed() const {
	return sqrt(vx*vx+vy*vy);
}


double CarSensorData::getYaw() const {
	double yaw= atan2(vy, vx);
	// Make sure the angle is between 0 and pi radians
	if (yaw<0)
		yaw += 2*pi();
	return yaw;
}

Car::Car(const FrenetCartesianConverter & converter_ini): s(.0), d(.0), x(.0), y(.0), yaw(.0), speed(.0), converter(converter_ini) {

}

unsigned Car::getLane() const {
	auto lane = floor(d/ConfigParams::lane_width);
	return static_cast<unsigned>(lane);
}

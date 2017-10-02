#include "coordinatesHandling.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include "spline.h"

using namespace std;

constexpr double pi() {
	return M_PI;
}


double deg2rad(double x) {
	return x * pi() / 180;
}


double rad2deg(double x) {
	return x * 180 / pi();
}


double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
		const vector<double> &maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (unsigned i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}

	}
	return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
		const vector<double> &maps_y) {

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi() / 4) { // TODO Keep an eye here!
		closestWaypoint++;
	}

	return closestWaypoint;

}

int NextWaypointExperimental(double x, double y, double theta, const vector<double> & maps_x, const vector<double> & maps_y)
// By Jeremy Owen
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double theta_pos = fmod(theta + (2*pi()),2*pi());
    double heading_pos = fmod(heading + (2*pi()),2*pi());
    double angle = abs(theta_pos-heading_pos);
    if (angle > pi()) {
        angle = (2*pi()) - angle;
    }

    // cout << "heading:" << heading << " diff:" << angle << endl;

    if(angle > pi()/2)
    {
        closestWaypoint = (closestWaypoint + 1) % maps_x.size();

    }

    return closestWaypoint;
}


void wrapForSpline(vector<double> & v, const unsigned n) {
	assert(n<=v.size());

	vector<double> first_n(begin(v), begin(v)+n);
	vector<double> last_n(end(v)-n, end(v));

	v.insert(begin(v), begin(last_n), end(last_n));
	v.insert(end(v), begin(first_n), end(first_n));
}


FrenetCartesianConverter::FrenetCartesianConverter(
		const vector<double> maps_s_init, const vector<double> maps_x_init,
		const vector<double> maps_y_init, const std::vector<double> maps_dx_init, const std::vector<double> maps_dy_init, const double max_s_init) :
		maps_s(maps_s_init), maps_x(maps_x_init), maps_y(maps_y_init), maps_dx(maps_dx_init), maps_dy(maps_dy_init), max_s(max_s_init) {
	constexpr unsigned n_to_wrap= 5;

	wrapForSpline(maps_s, n_to_wrap);
	for (unsigned i=0; i< n_to_wrap; ++i) {
		maps_s[i]-=max_s;
		maps_s[maps_s.size()-1-i]+= max_s;
	}

	wrapForSpline(maps_x, n_to_wrap);
	wrapForSpline(maps_y, n_to_wrap);
	wrapForSpline(maps_dx, n_to_wrap);
	wrapForSpline(maps_dy, n_to_wrap);


	spline_maps_x.set_points(maps_s, maps_x);
	spline_maps_y.set_points(maps_s, maps_y);
	spline_maps_dx.set_points(maps_s, maps_dx);
	spline_maps_dy.set_points(maps_s, maps_dy);
}


pair<double, double> FrenetCartesianConverter::getXY2(const double s,
		const double d) const {
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
			(maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x,y};
}


pair<double, double> FrenetCartesianConverter::getXY(const double s,
		const double d) const {

	// Wrap s around the [0, max_s] interval
	double wrapped_s = s;
	if (wrapped_s < 0)
		wrapped_s+=max_s;
	else if (wrapped_s >= max_s)
		wrapped_s-=max_s;

	double x0= spline_maps_x(wrapped_s);
	double y0= spline_maps_y(wrapped_s);
	double dx= spline_maps_dx(wrapped_s);
	double dy= spline_maps_dy(wrapped_s);
	double x= x0 + d*dx;
	double y= y0 + d*dy;
	return {x, y};
}


pair<double, double> FrenetCartesianConverter::getFrenet(double x, double y,
		double theta) const {
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s,frenet_d};
}

/**
 * Gets the heading of the road at a given `s` coordinate; it is an angle in radians between 0 and 2 times
 * pi, taken counter-clockwise, with 0 radians corresponding to due East.
 * @param s the 's' component of the Frenet road coordinate where the heading is desired.
 * @return the calculated road heading.
 */
double FrenetCartesianConverter::getRoadHeading(const double s) const {
	// Get the direction perpendicular to the road
	double dx= spline_maps_dx(s);
	double dy= spline_maps_dy(s);
	double theta=atan2(dy, dx);

	// Convert it to the road heading (rotate it 90 deg. counterclockwise).
	theta+=pi()/2; // This is between -pi/2 and 3/2*pi radians

	// Make sure the angle is between 0 and pi radians
	if (theta<0)
		theta = 2*pi()+theta;
	return theta;
}

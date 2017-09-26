#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}
double deg2rad(double x) {
	return x * pi() / 180;
}
double rad2deg(double x) {
	return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
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

	if (angle > pi() / 4) {
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
		const vector<double> &maps_x, const vector<double> &maps_y) {
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y) {
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

// Keep lane, prepare to change lane left, prepare to change lane right, change lane left, change lane right
enum struct CarState { KL, PLCL, PLCR, CLL, CLR };

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ =
			"/home/fanta/workspace/CarND-Path-Planning-Project/data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	auto lane = 1;  // Center lane

	auto ref_vel = .0;  // mph

	auto car_state = CarState::KL;

	h.onMessage(
			[&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &car_state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				//auto sdata = string(data).substr(0, length);
				//cout << sdata << endl;
				if (length && length > 2 && data[0] == '4' && data[1] == '2') {

					auto s = hasData(data);

					if (s != "") {
						auto j = json::parse(s);

						string event = j[0].get<string>();

						if (event == "telemetry") {
							// j[1] is the data JSON object

							// Main car's localization Data
							double car_x = j[1]["x"];
							double car_y = j[1]["y"];
							double car_s = j[1]["s"];
							double car_d = j[1]["d"];
							double car_yaw = j[1]["yaw"];
							double car_speed = j[1]["speed"];

							// Previous path data given to the Planner
							auto previous_path_x = j[1]["previous_path_x"];
							auto previous_path_y = j[1]["previous_path_y"];
							// Previous path's end s and d values
							double end_path_s = j[1]["end_path_s"];
							double end_path_d = j[1]["end_path_d"];

							// Sensor Fusion Data, a list of all other cars on the same side of the road.
							auto sensor_fusion = j[1]["sensor_fusion"];

							int prev_size = previous_path_x.size();

							if (prev_size > 0) {
								car_s = end_path_s;  // TODO Why??
							}

							/*
							 * States: KL, PLCL, PLCR, CLL, CLR
							 *
							 * >> General principle: try to keep the target speed, choosing a lane with no traffic, or the lane with the fastest
							 * traffic.
							 * Speed of traffic in your same lane is given by the speed of the car in front of you (if any);
							 * in a different lane, it is given by the speed of the car in front of the first space where you could move.
							 *
							 * A space is available to move into if it is long enough, keeping into account the car length and the
							 * reaction time given the speed of the car preceding the space, and some buffer (simplification: use reaction
							 * time for speed=target speed).
							 *
							 * >> State machine. If you are not changing lane, keep checking if you should, and in case take action accordingly;
							 * also, keep distance from the car in front of you (if any). Distance must keep in consideration reaction time
							 * and speed (simplification: always consider the reaction time for worst case, i.e. speed = target speed).
							 * If you are changing lane already, carry on with the manoeuvre until completed (improvement:
							 * abort the manoeuvre if it is getting dangerous).
							 */

							switch(car_state) {
							case CarState::KL:
							case CarState::PLCL:
							case CarState::PLCR:;

							}


							bool too_close= false;

							for (unsigned i=0; i < sensor_fusion.size(); ++i) {
								// Is the other car in my same lane?
								float d= sensor_fusion[i][6];
								if (d< 4+4*lane && d> 4*lane) {
									double vx= sensor_fusion[i][3];
									double vy= sensor_fusion[i][4];
									double check_speed= sqrt(vx*vx+vy+vy);
									double check_car_s= sensor_fusion[i][5];

									check_car_s+=((double)prev_size*.02*check_speed); // TODO fix cast
									if(check_car_s > car_s && check_car_s-car_s < 30) {
										// ref_vel= 29.5;
										too_close= true;

									}
								}
							}

							if (too_close) {
								ref_vel-=.224;
							}
							else if (ref_vel < 49.5) {
								ref_vel+=.224;
							}

							// A list of (x, y) waypoints, to be interpolated with a spline, more widely spaced than previous_path_x[] and previous_path_y[]
							vector<double> ptsx;
							vector<double> ptsy;

							// reference car pose (x, y, yaw)
							double ref_x = car_x;
							double ref_y = car_y;
							double ref_yaw = deg2rad(car_yaw);

							// If the previous path doesn't contain at least two points, use the current car pose as reference
							if (prev_size < 2) {
								// Path needs to be tangent to the car yaw
								double prev_car_x = car_x - cos(car_yaw);
								double prev_car_y = car_y - sin(car_yaw);

								ptsx.push_back(prev_car_x);
								ptsx.push_back(car_x);

								ptsy.push_back(prev_car_y);
								ptsy.push_back(car_y);
							}
							// Otherwise use the last to points of the previous path, to get to points defining a line tangent to the car yaw
							else {

								ref_x = previous_path_x[prev_size-1];
								ref_y = previous_path_y[prev_size-1];

								double ref_x_prev = previous_path_x[prev_size-2];
								double ref_y_prev = previous_path_y[prev_size-2];
								ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

								ptsx.push_back(ref_x_prev);
								ptsx.push_back(ref_x);

								ptsy.push_back(ref_y_prev);
								ptsy.push_back(ref_y);
							}

							vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
							vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
							vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

							ptsx.push_back(next_wp0[0]);
							ptsx.push_back(next_wp1[0]);
							ptsx.push_back(next_wp2[0]);

							ptsy.push_back(next_wp0[1]);
							ptsy.push_back(next_wp1[1]);
							ptsy.push_back(next_wp2[1]);

							for (unsigned i=0; i<ptsx.size(); ++i) {
								// Convert (x, y, theta) from universal to car reference system
								double shift_x= ptsx[i]-ref_x;
								double shift_y= ptsy[i]-ref_y;

								ptsx[i]= (shift_x*cos(-ref_yaw)-shift_y*sin(-ref_yaw));
								ptsy[i]= (shift_x*sin(-ref_yaw)+shift_y*cos(-ref_yaw));
							}

							// Instantiate the spline
							tk::spline s;
							s.set_points(ptsx, ptsy);

							// The path to be fed to the simulator
							vector<double> next_x_vals;
							vector<double> next_y_vals;

							// Begin by filling the path with all the points from the previous path
							for (unsigned i=0; i<previous_path_x.size(); ++i) {
								next_x_vals.push_back(previous_path_x[i]);
								next_y_vals.push_back(previous_path_y[i]);
							}

							// Break up the spline such that the car travels at the target velocity (with some approximation)
							double target_x=30.0;
							double target_y=s(target_x);
							double target_dist = sqrt(target_x*target_x+target_y*target_y);

							double x_add_on= .0;

							// Complete the planned path with new points
							for (unsigned i=1; i<=50-previous_path_x.size(); ++i) {
								double N= target_dist/(.02*ref_vel/2.24);
								double x_point= x_add_on+target_x/N;
								double y_point= s(x_point);

								x_add_on= x_point;

								double x_ref= x_point;
								double y_ref= y_point;

								// Rototranslation to go back to car (x, y, theta) reference system to the global reference system
								x_point= x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);// TODO: move to a subroutine
								y_point= x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

								x_point += ref_x;
								y_point += ref_y;

								next_x_vals.push_back(x_point);
								next_y_vals.push_back(y_point);
							}

							json msgJson;

							/*double x_add_on=0;

							 double dist_inc = 0.25;
							 for(int i = 0; i < 50; i++)
							 {
							 double next_s = car_s+(i+1)*dist_inc;
							 double next_d = 6.;
							 vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
							 // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
							 // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
							 next_x_vals.push_back(xy[0]);
							 next_y_vals.push_back(xy[1]);
							 } */

							// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
							msgJson["next_x"] = next_x_vals;
							msgJson["next_y"] = next_y_vals;

							auto msg = "42[\"control\","+ msgJson.dump()+"]";

							//this_thread::sleep_for(chrono::milliseconds(1000));
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

						}
					} else {
						// Manual driving
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}
			});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}

#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <cassert>
#include "Eigen/Core"
#include "Eigen/QR"

#include "coordinatesHandling.h"
#include "json.hpp"
//#include "spline.h"

using namespace std;
using namespace Eigen;

// for convenience
using json = nlohmann::json;

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

/* Convenient alias for a type, will hold the 6 coefficients of a quintic function, sorted
 * from the degree 0 coefficient to the degree 5: a0+a1*x+a2*x^2+a3*x^3+a4*x^4+a5*x^5 .
 */
typedef Matrix<double, 6, 1> Vector6d;

/**
 * Computes the jerk minimising trajectory for the given boundary conditions and time interval.
 * @param start the starting boundary conditions.
 * @param goal the ending boundary conditions.
 * @param t the time interval in seconds to reach the goal from start.
 * @return the coefficients of a quintic function, providing the requested trajectory, ordered
 * from the term of degree 0 to the term of degree 5.
 */
Vector6d computeJMT(const Vector3d start, const Vector3d goal, double t) {
	auto a0 = start[0];
	auto a1 = start[1];
	auto a2 = start[2] / 2.;

	Matrix3d A;
	A << pow(t, 3), pow(t, 4), pow(t, 5), 3 * pow(t, 2), 4 * pow(t, 3), 5
			* pow(t, 4), 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

	auto c0 = a0 + a1 * t + a2 * pow(t, 2);
	auto c1 = a1 + 2 * a2 * t;
	auto c2 = 2 * a2;

	Vector3d b;
	b << goal[0] - c0, goal[1] - c1, goal[2] - c2;

	Vector3d x = A.colPivHouseholderQr().solve(b);
	Vector6d result;
	result << a0, a1, a2, x;
	return result;
}

/**
 * Evaluates the quintic function with the given coefficients in `x`.
 * @param coeffs the quintic function coefficients, ordered from the term of degree 0 to the term of degree 5.
 * @param x the value for which to evaluate the quintic function.
 * @return the computed value.
 */
double evalQuintic(Vector6d coeffs, double x) {
	double result = coeffs[0] + coeffs[1] * x + coeffs[2] * pow(x, 2)
			+ coeffs[3] * pow(x, 3) + coeffs[4] * pow(x, 4)
			+ coeffs[5] * pow(x, 5);
	return result;
}

// Keep lane, prepare to change lane left, prepare to change lane right, change lane left, change lane right
enum struct CarState {
	KL, PLCL, PLCR, CLL, CLR
};

pair<double, double> universal2car_ref(const pair<double, double> xy,
		const double car_x, const double car_y, const double car_yaw) {

	double shift_x = xy.first - car_x;
	double shift_y = xy.second - car_y;

	double x_res = (shift_x * cos(-car_yaw) - shift_y * sin(-car_yaw));
	double y_res = (shift_x * sin(-car_yaw) + shift_y * cos(-car_yaw));

	return {x_res, y_res};
}

pair<double, double> car2universal_ref(const pair<double, double> xy,
		const double car_x, const double car_y, const double car_yaw) {
	double unrot_x = xy.first * cos(car_yaw) - xy.second * sin(car_yaw);
	double unrot_y = xy.first * sin(car_yaw) + xy.second * cos(car_yaw);

	double x_res = unrot_x + car_x;
	double y_res = unrot_y + car_y;

	return {x_res, y_res};
}

bool close_enough(const double a, const double b) {
	constexpr double tollerance = 0.001;
	if (abs(a - b) <= tollerance)
		return true;
	return false;
}


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
	constexpr double max_s = 6945.554;

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

	// auto lane = 1;  // Center lane
	// auto car_state = CarState::KL;

	auto prev_t = std::chrono::high_resolution_clock::now();
	long iterations = 1;
	Vector3d last_s_boundary_conditions;
	bool last_s_boundary_conditions_init = false;

	/**************************************/
	/* All measures below are in the I.S. */
	/**************************************/
	double prev_vel_s = 0;

	// double delta_t = 0.05;

	// Duration of the planned trajectory
	constexpr double planning_t = 5;

	// Desired cruise speed, that the car should try to attain and keep
	constexpr double target_speed = 15;

	// Maximum acceptable acceleration for the car, will try to reach it to get to target_sped in the shortest time
	constexpr double max_accel_s = 7;

	// Time interval between two consecutive waypoints, as implemented by the simulator
	constexpr double tick = 0.02;

	// When the planned trajectory yet to be run goes under this duration, extend it by planning a new trajectory
	constexpr double min_trajectory_duration = 0.5;

	FrenetCartesianConverter coord_conv(map_waypoints_s, map_waypoints_x,
			map_waypoints_y, map_waypoints_dx, map_waypoints_dy, max_s);

	// ofstream log_file;
	// log_file.open ("/home/fanta/workspace/CarND-Path-Planning-Project/data/log.txt");

	//bool jumped = false;

	h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

						/* Jump to the end of the track
						if (!jumped) {
							json msgJson;
							auto xy = coord_conv.getXY(max_s-400, 6);
							msgJson["next_x"] = {xy.first };
							msgJson["next_y"] = {xy.second};

							auto msg = "42[\"control\","+ msgJson.dump()+"]";

							//this_thread::sleep_for(chrono::milliseconds(1000));
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
							jumped= true;
							return;
						}*/

						auto current_t = std::chrono::high_resolution_clock::now();
						std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_t - prev_t);
						double delta_t = time_span.count();
						prev_t = current_t;

						// Main car's localization Data
						double car_x = j[1]["x"];  // m
						double car_y = j[1]["y"];  // m
						double car_s = j[1]["s"];  // m
						double car_d = j[1]["d"];  // m
						double car_yaw_deg = j[1]["yaw"];  // Degrees
						auto car_yaw = deg2rad(car_yaw_deg);  // Converted to radians
						double car_speed_mph = j[1]["speed"];  // mps
						auto car_speed= 1609.344*car_speed_mph/3600;  // Converted to m/s

						// Previous path data given to the Planner; item [0] is the closest to the car
						vector<double> previous_path_x = j[1]["previous_path_x"];
						vector<double> previous_path_y = j[1]["previous_path_y"];
						// Previous path's end s and d values (corresponding to the last element in previous_path_x[] and previous_path_y[]
						double end_path_s = j[1]["end_path_s"];
						// double end_path_d = j[1]["end_path_d"];

						// Sensor Fusion Data, a list of all other cars on the same side of the road.
						auto sensor_fusion = j[1]["sensor_fusion"];

						// Set starting boundary conditions for the first JMT to be calculated
						if (!last_s_boundary_conditions_init) {
							last_s_boundary_conditions[0] = car_s;
							last_s_boundary_conditions_init = true;
							assert(car_speed==.0);
						}

						// Determine road heading, and use it to compute the s and d components of the car velocity
						double road_h = coord_conv.getRoadHeading(car_s);
						double car_vel_s= car_speed*cos(car_yaw- road_h);
						double car_vel_d= -car_speed*sin(car_yaw-road_h);// d=0 on the yellow center line, and increases toward the outer of the track

						// Estimate s component of the car acceleration based on current and previous car velocity along s
						double car_accel_s= (car_vel_s-prev_vel_s)/ delta_t;
						prev_vel_s = car_vel_s;

						/* Fill in the path (waypoints) to be fed to the simulator. Start by copying all waypoints
						 * still unused at the current iteration.
						 */

						vector<double> next_x_vals(previous_path_x);
						vector<double> next_y_vals(previous_path_y);


						/* If the trajectory covers less than min_trajectory_duration seconds, then extend it by
						 * computing a new JMT and stitching it to the end.
						 */

						const double remaining_path_duration = previous_path_x.size()*tick;
						// cout << "Remaining " << remaining_path_duration << endl;
						if (remaining_path_duration < min_trajectory_duration) {
							cout << "Iteration# " << iterations << endl;
							cout << "s= " << car_s << "  road heading= " << rad2deg(road_h) << "  yaw= " << rad2deg(car_yaw) << endl;
							cout << "speed= " << car_speed << "  speed_s= "<< car_vel_s << "  speed_d= " << car_vel_d << endl;
							cout << "  accel_s= "<< car_accel_s << endl;

							// Determine boundary conditions for quintic polynomial (car trajectory in Frenet coordinates)

							Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
							Vector3d s_goal;// Goal conditions for s
							double proj_vel_s = s_start[1] + max_accel_s*planning_t;  // TODO take also car_vel_d into account
							if (proj_vel_s < target_speed)
								s_goal << s_start[0]+s_start[1]*planning_t+.5*max_accel_s*pow(planning_t,2), proj_vel_s, max_accel_s;
							else {
								double tx= (target_speed - s_start[1]) / max_accel_s;
								double s1= s_start[0] + s_start[1]*tx+.5*max_accel_s*pow(tx,2);
								double s2= (planning_t - tx) * target_speed;
								s_goal << s1+s2, target_speed, 0;
							}
							cout << "start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

							// Update the boundary conditions to be used at the beginning of the next JMT
							last_s_boundary_conditions = s_goal;
							if (last_s_boundary_conditions[0] >= max_s)
								last_s_boundary_conditions[0]-=max_s;

							// Compute the quintic polynomial coefficients, for the given boundary conditions and planning time interval
							auto sJMT = computeJMT(s_start, s_goal, planning_t);
							cout << "JMT= " << sJMT.transpose() << endl << endl;

							/* Sample waypoints from the trajectory at time intervals of duration tick,
							 * and store them in Cartesian (universal) coordinates. We want one waypoint at the end of every tick,
							 * from time 0 to time planning_t
							 */
							vector<pair<double, double>> wpoints; // Will hold the sampled waypoints
							vector<double> ss;
							unsigned n_planning_wpoints=static_cast<int>(round(planning_t / tick));
							double previous_s =-1;
							for (unsigned i_wpoint=0; i_wpoint<n_planning_wpoints; ++i_wpoint) {
								double wpoint_t= tick*(i_wpoint+1);
								double next_s= evalQuintic(sJMT, wpoint_t);
								ss.push_back(next_s);
								if (next_s < previous_s && !close_enough(next_s, previous_s))
									cout << "ERROR: backstep  next_s= " << next_s << "  previous_s= " << previous_s << endl;
								if (next_s < car_s && ! close_enough(next_s, car_s))
									cout << "ERROR: going backward  next_s= " << next_s << "  car_s= " << car_s << endl;
								double next_d= 6;
								auto xy= coord_conv.getXY(next_s, next_d);
								wpoints.push_back(xy);
							}
							assert(wpoints.size()==n_planning_wpoints);

							// Next add waypoints from the just computed JMT

							for (auto wpoint: wpoints) {
								next_x_vals.push_back(wpoint.first);
								next_y_vals.push_back(wpoint.second);
							}
						} // if (remaining_path_duration < min_trajectory_duration)

					json msgJson;

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					++iterations;

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
			// I guess this should be done more gracefully?
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

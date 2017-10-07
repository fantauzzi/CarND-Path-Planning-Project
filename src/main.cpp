#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <cassert>
#include <limits>
#include "Eigen/Core"
#include "Eigen/QR"
#include "coordinatesHandling.h"
#include "car.h"
#include "json.hpp"
#include "FSM.h"

using namespace std;
using namespace Eigen;

// for convenience
using json = nlohmann::json;

// Yes, the lane width
constexpr double lane_width = 4;

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

	unsigned lane = 1;  // Center lane
	auto car_state = CarState::KL;

	auto prev_t = std::chrono::high_resolution_clock::now();
	long iterations = 1;
	Vector3d last_s_boundary_conditions;
	Vector3d last_d_boundary_conditions;
	bool last_boundary_conditions_init = false;

	/**************************************/
	/* All measures below are in the I.S. */
	/**************************************/

	// Next two used to estimate accelerations; commented out and not using the acceleration at present
	// double prev_vel_s = 0;
	// double prev_vel_d = 0;
	// double delta_t = 0.05;
	// Duration of the planned trajectory
	constexpr double planning_t = 1;

	// Desired cruise speed, that the behaviour planning shall try to attain and keep
	constexpr double cruise_speed = 21.4579;

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

	constexpr double sensor_range= 300;

	// The speed the car tries to attain and maintain, can change at every iteration based on behaviour planning
	double target_speed = 21.4579;

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
						/* - Sense and predict, to anticipate where other cars are gonna be when I reach the end of the current JMT
						 * - Run the FSM and determine the state for the current iteration.
						 * - Based on the state, compute one or more trajectories.
						 * - Choose the trajectory with the lowest cost.
						 * - Send it to the simulator
						 */

						// Based on the state, compute one or more trajectories

						auto current_t = std::chrono::high_resolution_clock::now();
						std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_t - prev_t);
						// double delta_t = time_span.count();
						// double delta_t = 0.05;  // TODO decide if I want to keep this, or the actual measurement
						// cout << "Delta-t= " << delta_t << endl;
						prev_t = current_t;

						// Main car's localization Data
						double car_x = j[1]["x"];  // m
						double car_y = j[1]["y"];  // m
						double car_s = j[1]["s"];// m
						double car_d = j[1]["d"];// m
						double car_yaw_deg = j[1]["yaw"];// Degrees
						auto car_yaw = deg2rad(car_yaw_deg);// Converted to radians
						double car_speed_mph = j[1]["speed"];// mps
						auto car_speed= 1609.344*car_speed_mph/3600;// Converted to m/s

						// Previous path data given to the Planner; item [0] is the closest to the car
						vector<double> previous_path_x = j[1]["previous_path_x"];
						vector<double> previous_path_y = j[1]["previous_path_y"];
						// Previous path's end s and d values (corresponding to the last element in previous_path_x[] and previous_path_y[]
						// double end_path_s = j[1]["end_path_s"];
						// double end_path_d = j[1]["end_path_d"];

						// Sensor Fusion Data, a list of all other cars on the same side of the road.
						vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
						vector<carSensorData> cars;
						for (auto item: sensor_fusion)
							if (abs(item[5]-car_s) <= sensor_range)
								cars.push_back(carSensorData(item, coord_conv));

						/* If state is KL:
						 * - find closest vehicle preceding in same lane
						 * - if close enough, adjust target_speed to that vehicle speed
						 * - if slower than me, look for possibility to change lane, i.e.:
						 * - find closest vehicle preceding in adjacent lane;
						 * - find closest vehicle following in the same adjacent lane;
						 * - if two vehicles are out of the way, transition to CLL/CLR (if two lanes are available for change, choose
						 *   the one without traffic, or else the one with the fastest traffic)
						 * - if still in KL state, when necessary plan new trajectory to keep lane
						 */

						// Initialise last_boundary_conditions at the first iteration (beginning of the simulation)
						if (!last_boundary_conditions_init) {
							assert(car_speed==.0);
							last_s_boundary_conditions[0] = car_s;
							last_d_boundary_conditions[0] = car_d;
							last_boundary_conditions_init = true;
						}

						/* Fill in the path (waypoints) to be fed to the simulator. Start by copying all waypoints
						 * still unused at the current iteration.
						 */
						vector<double> next_x_vals(previous_path_x);
						vector<double> next_y_vals(previous_path_y);

						// Time left before the car will gobble up the current path (in seconds)
						const double remaining_path_duration = previous_path_x.size()*tick;

						switch(car_state) {
						case CarState::KL:
							// Find the closest vehicle in range preceding in the same lane (if any)
							auto closest_info= findClosestInLane({car_s, car_d}, cars, lane, true, lane_width);
							int closest_i= closest_info.first;  // Will be the position in cars[] of the found vehicle (if found)
							double closest_dist=  closest_info.second;

							/* If the distance is below a certain amount, set the target speed to the speed
							 * of the preceding car, less some margin; otherwise set the target speed to the max cruise speed
							 */
							bool consider_lane_change= false;
							if (closest_i >= 0 && closest_dist <= 50) {
								target_speed= min(sqrt(pow(cars[closest_i].vx,2)+pow(cars[closest_i].vy,2))*.99, cruise_speed);
								consider_lane_change= true;  // TODO not quite right, what if the preceding car is faster than cruise_speed?
								// cout << "Separation=" << closest_dist << " car#" <<closest_i << " target_speed=" << target_speed << endl;
							}
							else
								target_speed= cruise_speed;

							if (consider_lane_change) {
								cout << "Considering lane change." << endl;
								// Which lanes should we consider for a lane change?
								vector<unsigned> lanes;
								if (lane==1)
									lanes= {0u, 2u};
								else
									lanes= {1u};
								int new_lane= -1;  // Will be set to the best lane to change to, or left set to -1 if no suitable lane change is found
								double new_lane_speed= 0;

								for (auto the_lane: lanes) {
									// Find closest preceding vehicle (if any) in adjacent lane
									auto preceding = findClosestInLane({car_s, car_d }, cars, the_lane, true, lane_width);
									int preceding_i= preceding.first;
									double preceding_dist= preceding.second;
									if (preceding_i >=0)
										cout << "Found preceding car in lane " << the_lane << " with distance " << preceding_dist << endl;
									auto following = findClosestInLane({car_s, car_d }, cars, the_lane, false, lane_width);
									int following_i= following.first;
									double following_dist= following.second;
									if (following_i >=0)
										cout << "Found following car in lane " << the_lane << " with distance " << following_dist << endl;
									// Is this lane change viable?
									bool viable= true;
									if (following_i >=0 && following_dist < 10)
										viable= false;
									else if (preceding_i >=0 && preceding_dist <15)
										viable= false;
									if (viable && (preceding_i<0 || (preceding_i>=0 && cars[preceding_i].getSpeed() > new_lane_speed))) {
										new_lane= the_lane;
										new_lane_speed= (preceding_i >= 0)? cars[preceding_i].getSpeed() : cruise_speed;
									}

								}
								//if (new_lane>=0)
								//	cout << "Decided for lane " << new_lane << " with speed " << new_lane_speed << endl;
							}
						}

						if (remaining_path_duration < min_trajectory_duration) {
							cout << "Iteration# " << iterations << endl;
							cout << "s=" << car_s << " d=" << car_d << " yaw=" << rad2deg(car_yaw) << endl;
							cout << "speed=" << car_speed << endl;

							// Determine boundary conditions for quintic polynomial (car trajectory in Frenet coordinates)

							Vector3d s_start = last_s_boundary_conditions;// Initial conditions for s
							Vector3d s_goal;// Goal conditions for s
							int a_sign = (target_speed > s_start[1])? 1 : -1;
							double proj_vel_s = s_start[1] + a_sign*max_accel_s*planning_t;
							if ((a_sign > 0 && proj_vel_s < target_speed) || (a_sign < 0 && proj_vel_s > target_speed))
								s_goal << s_start[0]+s_start[1]*planning_t+.5*(a_sign)*max_accel_s*pow(planning_t,2), proj_vel_s, a_sign*max_accel_s;
							else {
								double tx= a_sign*(target_speed - s_start[1]) / max_accel_s;
								double s1= s_start[0] + s_start[1]*tx+.5*a_sign*max_accel_s*pow(tx,2);
								double s2= (planning_t - tx) * target_speed;
								s_goal << s1+s2, target_speed, 0;
							}
							cout << "s start and goal" << endl << s_start.transpose() << endl << s_goal.transpose() << endl;

							Vector3d d_start = last_d_boundary_conditions; // Initial conditions for d
							Vector3d d_goal;// Goal conditions for d
							d_goal << 2+lane*4, 0, 0;
							cout << "d start and goal" << endl << d_start.transpose() << endl << d_goal.transpose() << endl;

							// Update the boundary conditions to be used at the beginning of the next JMT
							last_s_boundary_conditions = s_goal;
							if (last_s_boundary_conditions[0] >= max_s)
								last_s_boundary_conditions[0]-=max_s;
							last_d_boundary_conditions = d_goal;

							// Compute the quintic polynomial coefficients, for the given boundary conditions and planning time interval
							auto sJMT = computeJMT(s_start, s_goal, planning_t);
							cout << "sJMT= " << sJMT.transpose() << endl << endl;
							auto dJMT = computeJMT(d_start, d_goal, planning_t);
							cout << "dJMT= " << dJMT.transpose() << endl << endl;

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
								double next_d= evalQuintic(dJMT, wpoint_t);
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



						/* If state is CLL/CLR:
						 * - when necessary, compute new trajectory to change lane
						 */

						/* Find the closest car preceding in the same lane, if any within sensors range
						 */

						// Determine road heading, and use it to compute the s and d components of the car velocity
						/*double road_h = coord_conv.getRoadHeading(car_s);
						double car_vel_s= car_speed*cos(car_yaw- road_h);
						double car_vel_d= -car_speed*sin(car_yaw-road_h);// d=0 on the yellow center line, and increases toward the outer of the track*/

						/* If the trajectory covers less than min_trajectory_duration seconds, then extend it by
						 * computing a new JMT and stitching it to the end.
						 */

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

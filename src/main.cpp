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
#include "coordinatesHandling.h"
#include "Car.h"
#include "json.hpp"
#include "FSM.h"
#include "ConfigParams.h"
#include "trajectory.h"

using namespace std;
using namespace Eigen;
using namespace ConfigParams;

// for convenience
using json = nlohmann::json;

// Yes, the lane width
// constexpr double lane_width = 4;

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

// Keep lane, prepare to change lane left, prepare to change lane right, change lane left, change lane right
enum struct CarState {
	KL, PLCL, PLCR, CLL, CLR
};

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

	auto prev_t = std::chrono::high_resolution_clock::now();
	long iterations = 1;
	Vector3d last_s_boundary_conditions;
	Vector3d last_d_boundary_conditions;
	bool last_boundary_conditions_init = false;

	FrenetCartesianConverter coord_conv(map_waypoints_s, map_waypoints_x,
			map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

	// ofstream log_file;
	// log_file.open ("/home/fanta/workspace/CarND-Path-Planning-Project/data/log.txt");

	auto converter= FrenetCartesianConverter(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
	Car car(converter);  // TODO make Car a proper class, with a xtor: it cannot exist without a converter (see below)!
	vector<CarSensorData> cars;
	unique_ptr<FSM_State> pState= make_unique<KeepLane>(car, cars);

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
						// double delta_t = 0.05;
						// cout << "Delta-t= " << delta_t << endl;
						prev_t = current_t;

						// Main car's localization Data
						car.x = j[1]["x"];  // m
						car.y = j[1]["y"];  // m
						car.s = j[1]["s"];// m
						car.d = j[1]["d"];// m
						double car_yaw_deg = j[1]["yaw"];// Degrees
						car.yaw = deg2rad(car_yaw_deg);// Converted to radians
						double car_speed_mph = j[1]["speed"];// mps
						car.speed= 1609.344*car_speed_mph/3600;// Converted to m/s

						/* Previous path data given to the Planner; item [0] is the closest to the car.
						 * Taking a de-tour to update these data into the car object, to make the compiler happy.
						 */
						vector<double> car_path_x = std::move(j[1]["previous_path_x"]);
						car.path_x= std::move(car_path_x);
						vector<double> car_path_y = std::move(j[1]["previous_path_y"]);
						car.path_y= std::move(car_path_y);

						/* Previous path's end s and d values (corresponding to the last element in car.path_x[] and car.path_y[] .
						 * Commented out because not used.
						 */
						// double end_path_s = j[1]["end_path_s"];
						// double end_path_d = j[1]["end_path_d"];

						// Sensor Fusion Data, a list of all other cars on the same side of the road.
						vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
						cars.clear();
						for (auto item: sensor_fusion)
							if (abs(item[5]-car.s) <= sensor_range)
								cars.push_back(CarSensorData(item, coord_conv));

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
							assert(car.speed==.0);
							pState->initBoundaryConditions({car.s,  0, 0}, {car.d, 0, 0});
							last_boundary_conditions_init = true;
						}

						/* Fill in the path (waypoints) to be fed to the simulator. Start by copying all waypoints
						 * still unused at the current iteration.
						 */
						vector<double> next_x_vals(car.path_x);
						vector<double> next_y_vals(car.path_y);

						// Time left before the car will gobble up the current path (in seconds)
						const double remaining_path_duration = car.path_x.size()*tick;

						// =============================

						bool state_changed;
						do {
							state_changed=false;
							auto pTmp= pState->getNextState(car, cars);
							if (pState.get() != pTmp) {
								pState.reset(pTmp);
								state_changed= true;
							}
						} while(state_changed);
						if (remaining_path_duration < min_trajectory_duration) {
							cout << endl << "Iteration# " << iterations << endl;
							cout << "s=" << car.s << " d=" << car.d << " yaw=" << rad2deg(car.yaw) << endl;
							cout << "speed=" << car.speed << endl;

							// Determine boundary conditions for quintic polynomial (car trajectory in Frenet coordinates)

							auto polynomials= pState->generateTrajectory();
							auto sJMT= polynomials.first;
							auto dJMT= polynomials.second;

							/* Sample waypoints from the trajectory at time intervals of duration tick,
							 * and store them in Cartesian (universal) coordinates. We want one waypoint at the end of every tick,
							 * from time 0 to time planning_t
							 */
							vector<pair<double, double>> wpoints; // Will hold the sampled waypoints
							vector<double> ss;
							unsigned n_planning_wpoints=static_cast<int>(round(pState->getPlanningTime() / tick));
							// double previous_s =-1;
							for (unsigned i_wpoint=0; i_wpoint<n_planning_wpoints; ++i_wpoint) {
								double wpoint_t= tick*(i_wpoint+1);
								double next_s= evalQuintic(sJMT, wpoint_t);
								ss.push_back(next_s);
								// if (next_s < previous_s && !close_enough(next_s, previous_s))
									// cout << "ERROR: backstep  next_s= " << next_s << "  previous_s= " << previous_s << endl;
								// if (next_s < car.s && ! close_enough(next_s, car.s))
									// cout << "ERROR: going backward  next_s= " << next_s << "  car.s= " << car.s << endl;
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
						auto current_t_end = std::chrono::high_resolution_clock::now();
						std::chrono::duration<double> iter_time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_t_end - current_t);
						cout << "Computation time(s): " << iter_time_span.count() << endl;

						} // if (remaining_path_duration < min_trajectory_duration)

						// Determine road heading, and use it to compute the s and d components of the car velocity
						/*double road_h = coord_conv.getRoadHeading(car.s);
						double car_vel_s= car.speed*cos(car.yaw- road_h);
						double car_vel_d= -car.speed*sin(car.yaw-road_h);// d=0 on the yellow center line, and increases toward the outer of the track*/

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

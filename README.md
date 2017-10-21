# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Credits

I have used the spline function provided [here](http://kluge.in-chemnitz.de/opensource/spline/) and [Eigen 3.3.4](http://eigen.tuxfamily.org/index.php?title=Main_Page). Source code for both are incuded in the present repository.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Car Model Path and Generation

When the simulation starts, I generate a path to follow and send it to the simulator. The first path originates from the car, that is initially stopped, and extends in front of it. The simulator drives the car along the path and sends me periodically information on the car state, what is reported by sensors, and what part of the path has yet to be run.

When the car is close enough to the end of the planned path (0.2 seconds, with current settings), I generate a new path, starting from the end of the current path, and send it to the simulator.

The beginning of the simulation is the only time when I compute the path starting from the car position; after that, I compute a new path starting from the end of the current one or, in other words, I extend the current path.  

A path is a trail of waypoints, that the car will gobble up at a rate of one waypoint every 0.02 seconds. Therefore, by setting the distance between waypoints, I also determine the car velocity.

A Finite State Machine (FSM) encodes the car behavior. It has three possible states:
- Keeping Lane (KL), when the car can attend its cruise speed, unhampered by other vehicles.
- Following Car (FC), when the car is keeping distance from a preceding vehicle in the same lane.
- Changing Lane (CL), when the car is moving from one lane to an adjacent one.

At every iteration (i.e. update from the simulator), I determine if the car should remain in the current state, or switch to a different one, based on a set of rules. For instance, if the car is in FC (Following Car)), and a nearby lane has no traffic, or the traffic is faster, and there is a gap to merge in, then switch to CL (Change Lane) state.

When it is time to compute a new path, extending the current one, I do it based on the current FSM state. I first determine where the path should end, say as forward as possible in the current lane (KL), as close as possible to a set distance from the preceding vehicle (FC), or forward and in an adjacent lane (CL). The exact position of the path end is based on a simple [kinematic model](https://www.khanacademy.org/science/physics/one-dimensional-motion/kinematic-formulas/a/what-are-the-kinematic-formulas) of the car, which is good enough for driving along the highway.

I then calculate a Jerk Minimising Trajectory (JMT), which is basically a quintic polynomial, going from the beginning of the new path (i.e. the end of the previous one), to its end. The JMT ensures that the car will be in a set position, velocity and acceleration at the end-points of the path, and also gives guarantees of continuity of the position and its first two derivatives. However, it does not give any other guarantee  (beside continuity) about velocity, acceleration and jerk *between* the endpoints.

Given a JMT, the car might exceed along the path the speed limit, or any set limit on acceleration and jerk. They are imposed because of physical limitations, safety and comfort. 

The way I addressed this is to generate a number of random JMTs in a neighbor of the one just computed, by perturbing its goal end-point by random amounts with uniform distribution. I then choose the best JMT based on a cost function. The cost considers the car velocity, acceleration and jerk at every waypoint along the trajectory.

A limit of this approach is that, once committed to a path, it is not possible to change it to adapt to sudden new conditions. For instance, when another car cuts in front of the driven car without leaving enough distance, it is not possible to hit on the breaks right away; instead, I first need to reach the end of the current path.

Other possible improvements consist in smarter and more flexible behavior, for example changing speed to match traffic in an adjacent lane and facilitate lane change.


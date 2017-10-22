# Path Planning Project
Self-Driving Car Engineer Nanodegree Program

![image2]

## Goals
In this project my goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. I am provided with the car's localization and sensor fusion data, and also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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
   
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

See file `README_sim_info.txt` for more information on how the simulation works.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Credits

I have used the spline function provided [here](http://kluge.in-chemnitz.de/opensource/spline/) and [Eigen 3.3.4](http://eigen.tuxfamily.org/index.php?title=Main_Page). Source code for both are included in the present repository.

[//]: # (Image References)

[image1]: ./Path_planning_FSM.png "Finite State Machine"
[image2]: ./screenshot.png "Simulator screenshot"

---

## Path Generation and Car Model

When the simulation starts, I generate a path to follow and send it to the simulator. The first path originates from the car, that is initially stopped, and extends in front of it. The simulator drives the car along the path and sends me periodically information on the car state, what is reported by sensors, and what part of the path has yet to be run.

When the car is close enough to the end of the planned path (0.2 seconds, with current settings), I generate a new path, starting from the end of the current path, and send it to the simulator.

The beginning of the simulation is the only time when I compute the path starting from the car position; after that, I compute a new path starting from the end of the current one or, in other words, I extend the current path.  

A path is a trail of waypoints, that the car will gobble up at a rate of one waypoint every 0.02 seconds. Therefore, by setting the distance between waypoints, I also determine the car velocity.

A Finite State Machine (FSM) encodes the car behavior. It has three possible states:
- Keep Lane (KL), when the car can attend its cruise speed, unhampered by other vehicles.
- Follow Vehicle (FV), when the car is keeping distance from a preceding vehicle in the same lane.
- ChangeLane (CL), when the car is moving from one lane to an adjacent one.

![image1]

At every iteration (i.e. update from the simulator), I determine if the car should remain in the current state, or switch to a different one, based on a set of rules. For instance, if the car is in FV (Follow Vehicle), and a nearby lane has no traffic, or the traffic is faster, and there is a gap to merge in, then switch to CL (Change Lane) state.

Note that the state may switch multiple times in a row, during the iteration. E.g. at the end of a lane change the state goes from CL to KL, but if there is an obstructing (slower) vehicle on the new lane, then the state changes to FV right away. 

When it is time to compute a new path, extending the current one, I do it based on the current FSM state. I first determine where the path should end, say as forward as possible in the current lane (KL), as close as possible to a set distance from the preceding vehicle (FV), or forward and in an adjacent lane (CL). The exact position of the path end is based on a simple [kinematic model](https://www.khanacademy.org/science/physics/one-dimensional-motion/kinematic-formulas/a/what-are-the-kinematic-formulas) of the car, which is good enough for driving along the highway.

I then calculate a Jerk Minimising Trajectory (JMT), which is basically a quintic polynomial, going from the beginning of the new path (i.e. the end of the previous one), to its end. The JMT ensures that the car will be in a set position, velocity and acceleration at the end-points of the path, and also gives guarantees of continuity of the position and its first two derivatives. However, it does not give any other guarantee  (beside continuity) about velocity, acceleration and jerk *between* the endpoints.

Given a JMT, the car might exceed along the path the speed limit, or any set limit on acceleration and jerk. They are imposed because of physical limitations, safety and comfort. 

The way I addressed this is to generate a number of random JMTs in a neighbor of the one just computed, by perturbing its goal end-point by random amounts with uniform distribution. I then choose the best JMT based on a cost function. The cost considers the car velocity, acceleration and jerk at every waypoint along the trajectory.

Note that all trajectories calculations are done in a Frenet reference system, as it is much simpler than in Cartesian coordinates. The generated path waypoints are then converted into Cartesian coordinates, as expected by the simulator.

A limit of this approach is that, once committed to a path, it is not possible to change it to adapt to sudden new conditions. For instance, when another car cuts in front of the driven car without leaving enough distance, it is not possible to hit on the breaks right away; instead, I first need to reach the end of the current path.

Other possible improvements consist in smarter and more flexible behavior, for example changing speed to match traffic in an adjacent lane and facilitate lane change, and allowing to go across two lanes at a time.

## Requirements

In this section I list the project requirement criteria as given by [Udacity](https://review.udacity.com/#!/rubrics/1020/view), and indicate how I addressed them.

### The code compiles correctly.

Code can be compiled following instructions provided above.

### The car is able to drive at least 4.32 miles without incident.

I have made available a video where the car drives for more than 4.32 miles without accidents. The simulator reports on-screen the distance driven without accidents.

### The car drives according to the speed limit.

The car tries to keep a cruise speed of 45.6 mph (against a speed limit of 50 mph), as configured by parameter `cruise_speed` in file `ConfigParams.cpp`. Also, the cost function used to evaluate generated trajectories penalises trajectories that, at any intermediate point, reach or exceed 50 mph; see parameter `speed_limit` in `ConfigParams.cpp`.

Implementation of the cost function is in `cost()` in file `FSM.cpp`. In the same file, `FSM_State::generateTrajectory()` calls `cost()` to evaluate generated trajectories. That member function also tries to adjust for mismatches between distances (and therefore, velocities) measured in Frenet and measured in Cartesian coordinates. While the two match where the road is straight, on the outer lane of bends the distance between two points along the lane is significantly shorter when measured in Frenet as opposed to Cartesian coordinates.   

### Max Acceleration and Jerk are not Exceeded.

Member functions `computeGoalBoundaryConditions()` of classes `KeepLane`, `FollowCar` and `ChangeLane` (see file `FSM.cpp`) set the boundary conditions for the trajectory to be generated in such a way that constraints of max acceleration (and velocity) are respected at the trajectory end-points. Adoption of a quintic polynomial for the trajectory ensures that [jerk is minimised](http://www.shadmehrlab.org/book/minimum_jerk/minimumjerk.htm).

Member function `FSM_State::generateTrajectory()` samples randomly generated trajectories from the given boundary conditions and evaluate them with the cost function, in order to find the trajectory with the preferable acceleration and jerk throughout.    
### Car does not have collisions.

Member function `computeGoalBoundaryConditions()` of class `FollowCar` tries to keep a set distance from the preceding car, configured by parameter `safe_distance` in file `ConfigParams.cpp`. It does so by applying kinematic equations to the motion of the car, and to predict the motion of the preceding car.

In the same class, member function `getNextState()` verifies that there is enough space in an adjacent lane before switching state to CL, and therefore before initiating a lane change. To do so, it considers the closest car (if any) preceding and following on that lane, and predicting their positions at the beginning and end of the maneuver.  

### The car stays in its lane, except for the time between changing lanes.

Boundary conditions of the trajectories are set in such a way that the car tries to keep a constant `d` coordinate, and therefore the same lane, except when directed to change lane. 

Member function `computeGoalBoundaryConditions()` of class `ChangeLane` sets boundary conditions in such a way that lane change is completed in no more than 1.8 seconds, as configured by parameter `planning_t_CL` in `ConfigParams.cpp`.

### The car is able to change lanes

The car changes lane when it is obstructed by a slower going vehicle in the same lane, and there is an adjacent lane offering better conditions.

In order to begin lane change to a given lane, the following conditions must be met:

- the given lane has no preceding car in range, or the preceding car is faster than the obstructing car, or is significantly more far away than the obstructing car;

- the preceding car in the given lane (if any) must be far enough to change lane without risking a collision;

- the following car in the given lane (if any) must be far enough, and also its predicted position at the end of the maneuver must be far enough, to change lane without risking a collision.  

The rules above are coded in 'FollowCar::getNextState()'.

### There is a reflection on how to generate paths.

See section `Path Generation and Car Model` in this same `README`.

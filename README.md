# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Overview

The purpose of this project is to implement a path planning algorithm, which drives our car. We have highway environment on the simulator provided by Udacity. We are receiving position and velocity information of our car as well as of nearby car from telemetry and sensor fusion respectively. The output of our path planner algorithm is a set of points that car is going to traverse in subsequent 0.02 seconds. the communication between the simulator and the path planner is done using WebSocket.

### Reflection

We have started with the code provided in the seed project. In which we learn how to run our car, how to follow a lane, how to accelerate slowly and how to make smooth lane transition. We have added two additional functions and they as follows:

### Understanding the vicinity (Line 256 to 288)
Here, we learn about how the other close cars are behaving. The first thing we check if there is any car in front of us because that is the only motivation to switch lane in our simulation(although not in real life). After that, we are also capturing the location of the closest car on both sides at the next timestamp. We are capturing this information to gauge how the other nearby cars are positioned in order to make lane change decision.

### Decision making (line 291 to 330)
After capturing all the information about the surroundings, we are ready to make a decision for our car. If there is any car in front of our car, then we look to switch the lane. For that we also look for our current lane, if we are at the extreme left or extreme right then the only option we have is toward centre lane. In this scenario, we look for the position of other cars in the other lane. If the transition is safe, we make it; if not we deaccelerate and wait for the clearance. But if we are in the centre lane, we have 2 options i.e., either to go right or left. First, we see whether it is safe to change any lane and after that, we look which way is safest of both and make the transition.

### Area of improvement
When a car is ahead of our car, we have to make a decision to change lane or deaccelerate. In case we are not changing lane because of cars in the target lane then we have to follow the car ahead, which is moving at a slower speed. In that case, our ride is not smooth because we are deaccelerating and accelerating. The better approach would be to deaccelerate and match the velocity of the car ahead.

The logic which I have written is very specific for the project. We can generalize it my building multiple trajectories and associated cost for each based on the feasibility, safety and comfort level, after that we can choose the best possible trajectory to follow.

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

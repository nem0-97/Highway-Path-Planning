# Highway Path Planning
This code as is will interact with the Simulator found [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)
## Dependencies:
##### To build and run as is this project requires:

###### To Run:
  1. [Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)
  2. [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)

###### To Build:
  3. cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
  4. make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  5. gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Running the Code:

The main program can be built and run by running the following commands from the project's top directory(this has already been done):

If no changes made to code just run:
1. Run it: `cd build && ./path_planning `
(make sure run it from build directory so it can find `../data/highway_map.csv` in `main.cpp`)

Or build with changes and run:

1. Remove old build directory and make new one: `rm -r build && mkdir build && cd build`
2. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
3. Run it: `./path_planning`

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

##### INPUT: values provided by the simulator to the c++ program
//info about car's state
`double car_x`
`double car_y`
`double car_s`
`double car_d`
`double car_yaw`
`double car_speed`

// Remaining points from path previously given to the simulator by the planner
`vector<double> previous_path_x`
`vector<double> previous_path_y`

// Previous path's end s and d values
`double end_path_s`
`double end_path_d`

// Sensor Fusion Data, a list of all other cars on the same side of the road
`vector<vector<double>> sensor_fusion`

//each vector represents a car as `[id,x,y,vx,vy,s,d]`


##### OUTPUT: values provided by the c++ program to the simulator
//The x and y values for the simulator to use to control car to follow
//(points are spaced out .02 seconds between each point)
`vector<double> next_x_vals`
`vector<double> next_y_vals`

## Code Structure:
In src the following files:
 1. main.cpp:

 Uses sensor fusion data about surrounding cars and uses it to decide whether and how much to speed up, slow down, change lanes without too much acceleration or jerk and staying under the speed limit. Then uses car's position and previous paths if available to use spline.h to generate a trajectory that satisfy the behaviors decided upon.

 2. spline.h:

 Handles creating smooth trajectories that we can sample from for points to create our path from, file is from: [spline.h](http://kluge.in-chemnitz.de/opensource/spline/Handles). We provide it with 5 anchor points to create the spline from, the first 2 are either car's position and a point it could have traveled through based on it's yaw or the last 2 points left from the previous path we generated which the simulator returned. The remaining 3 points are up ahead of the car and in the center of the lane we want our car to be traveling in. The anchor points are transformed from global map (x,y) coords to (x,y) coords relative to the car before passed in to define the spline. Once the spline is created we first add whats left over from the previous path to the next path that we are goin to give the simulator, we then sample the spline for more points in order to make the path have 50 points(1 second into the future). These points are sampled from the spline at x intervals calculated using the speed we decided we want to be traveling at, the sampled points then are transformed from (x,y) coords relative to the car to global map (x,y) coords which are added to the end of the path we send the simulator.

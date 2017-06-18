# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model Predictive Control

The goal for this project is to implement a Model Predictive Controller (MPC)
to determine throttle/braking and steering values to successfully 
drive a car around Udacity's simulator track. 
An MPC relies on a model, a cost function and a time window (time slots). 
Its objective is to minimize the cost for the entirety of time slots. 
 
### Kinematic Model
 
In this project, we're describing the car using the kinematic model. The 
state vector for this model is comprised of four elements: x coordinate, 
y coordinate, orientation (psi) and velocity (v). Besides these four 
elements, our model also keeps track of the cross track error (how far 
the car is from the target position) and the orientation error (the 
difference between the target orientation and current orientation). 
The update equations for all of these elements are below:
 
    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

, where psides is the 'desired psi', the tangential angle of the desired 
trajectory evaluated at x[t].

The goal of our MPC is to determine steering angle and 
throttle value for the car (a negative throttle is the equivalent of 
braking) that minimize the total cost function.

### Waypoints and coordinate transform

In order to determine appropriate values for the actuators, it's first 
necessary to know what's the intended trajectory for the car. This is 
given by the simulator as a set of waypoints in map coordinates. In order 
to ease the process of determining actuator values, we first convert 
the waypoints to car coordinates. Considering that the car position is at 
the origin of its coordinate system and that the current car orientation 
determines the position for the x axis (so that the y axis is to the 
left of the car), we can use the following equations for the 
coordinates transformation:

    delta_x = waypoint_x_map - car_x_map
    delta_y = waypoint_y_map - car_y_map
    waypoint_x_car = (delta_x * cos(0 - psi)) - (delta_y * sin(0 - psi))
    waypoint_y_car = (delta_x * sin(0 - psi)) + (delta_y * cos(0 - psi))

I then fit a 3rd order polynomial to the transformed waypoints. This 
polynomial is the intended trajectory for the car.

### Cost function and how much to look into the future

Prior to running the linear optimizer one must define what to optimize. I 
opted for a cost function that takes into account 7 elements:

* Cross track error
* Orientation error
* Difference between reference velocity (90/MPH) and current velocity
* How much the car is steering
* How much the car is accelerating/braking
* Delta between previous and current steering
* Delta between previous and current throttle

Each of these items are meant to make the car trajectory smooth, maximize 
speed and keep the car close to the intended trajectory. Tuning these 
parameters (each is multiplied by an individual cost factor) was an 
empirical process - it's very interesting to observe how each of these 
cost elements impact driving quality in the simulator. 
The selected cost for each element can be seen on the MPC class.
 
Another element to tune is how many actuation points are used by the  
optimizer and how far apart these points are. This is controlled by 
parameters <code>N</code> and <code>dt</code>, respectively. If both are fixed, then the distance 
for which actuators are predicted and optimized is variable. I decided, 
instead, to apply a fixed distance of 15 meters and to use 12 actuation 
points. Therefore, <code>dt</code> depends on the car's current speed.

### Latency

The last element to take into account in the MPC is how to deal with 
latency. There's a time gap between sending new steering angle and throttle 
values to the car and having these values transmitted to the engine and 
wheels (100ms in this specific project). One way to deal with this problem 
is to manipulate the state vector that serves as input for MPC. By 
calculating what x, y, orientation and velocity will be at the time the 
actuators impact the car, we can approximate what it would be to drive 
without actuators latency. Latency equations are below (notice that 
some elements of the equation are cancelled out by the coordinate 
transform and are there only to completely illustrate the formulae):

    pred_psi = psi + (((v * steer) / mpc.Lf()) * latency_in_s) - psi;          
    pred_x = px + (v * cos(-pred_psi) * latency_in_s) - px;
    pred_y = py + (v * sin(-pred_psi) * latency_in_s) - py;
    pred_v = v + (current_throttle * latency_in_s);


### Results

With all that applied to the MPC, the car can successfully drive at 
an average of approximately 60/MPH. There are still several improvements 
that coulde be made. For instance, the car has a tendency of 
applying the brakes at the end of a curve, when it's generally better to 
do so prior to entering the curve. Choosing a different model for the car 
or further tuning the cost function could improve the results. 
Another option for tuning the cost function is to use a gradient ascent 
method - convergence time might be a problem, however, given how many 
parameters are available. Modifying the number of actuation points also 
has an enormous impact in driving. Right now it is a fixed value but 
it might be beneficial to use more or less points depending on the region 
the car is driving (curved/straight road, fast/slow speeds).

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

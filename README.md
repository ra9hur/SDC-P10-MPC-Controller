## Project Description
The purpose of this project is to develop a nonlinear model predictive controller (NMPC) to steer a car around a track in a simulator. This project has already been implemented using Behavioural Cloning in Term 1 and PID controller in Term 2.

---

![mpc1](https://user-images.githubusercontent.com/17127066/27439585-ebb74b46-5785-11e7-9ed2-29d1de8227b7.png)
Source: [iopscience](http://cdn.iopscience.com/images/0964-1726/21/5/055018/Full/sms399992f2_online.jpg)

![mpc2](https://user-images.githubusercontent.com/17127066/27439584-eb9cc7bc-5785-11e7-88da-2aad8e1eb509.png)
Source: [wikipedia](https://en.wikipedia.org/wiki/Model_predictive_control)

Model process controllers primarily solves an optimization problem and mainly has three parts 

- cost/objective function
- design variables
- constraints (inequality and equality).

A few possible objectives that could be defined are:

- Minimize the difference between model predicted values and the reference trajectory
- Minimize the difference actual velocity and reference velocity
- Minimize change in steering angle/throttle between time-steps for smooth driving

The IPOPT optimizer used in this project tries to find the minimum of the objective function by varying the values of the design variables while making sure all the constraints are satisfied. 

Design variables are the vehicle states [x, y, psi, v, cte, epsi] and control variables [steering angle delta, throttle a] at every time step until the prediction horizon. There are [x0, y0 , psi0, v0, cte0, epsi0] at t=0, [x1, y1, psi1, v1, cte1, epsi1] at t=0.1 and so on till the prediction horizon. If N=20 is the number of time-steps in the prediction horizon, the total number of variables would be,

     N * 6 + (N-1) * 2 = 20 * 6 + (20-1) * 2 = 158 variables.

The optimizer does not know about the restrictions on these variables. A few of those restrictions are:

- Easiest way to minimize the cross-track error is for the vehicle to "teleport" from it's starting position to be directly on the track and just stay there. The optimizer doesn't know that the car cannot teleport from the current position to the target or that it's velocity cannot instantaneously increase to the desired value.
- Car cannot take arbitrary path. It has to move on the left/right side of the road depending on country rules.
- Run the engine such that emissions are within limits. Fuel/air ratio has to be carefully maintained.
- To minimize distance, it cannot be crossing streets while calculating euclidian distance. It might have to calculate manhatten distance instead to minimize.

The only way to instruct optimizer on these restrictions is through constraints. IPOPT accepts constraints in a certain format. In case of equality constraints it expects functions that evaluate to zero when the constraint is satisfied. So the expression below is constraining the variables x1, x0, v0 and psi0 such that:

      x1 - (x0 + v0 * CppAD::cos(psi0) * dt) = 0

This ensures that the dynamics of the system are satisfied between the two time-steps for the state 'x'. Similarly, constraints to enforce the kinematics of every state at every time-step are added using vehicle model equations.
      
      y1 - (y0 + v0 * CppAD::sin(psi0) * dt) = 0
      psi1 - (psi0 - (v0/Lf * delta0 * dt)) = 0
      v1 - (v0 + a0 * dt) = 0
      cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)) = 0
      epsi1 - ((psi0 - psides0) + v0 * delta0 * dt / Lf) = 0

There are other constraints that are inequality constraints and also a special form of inequality constraint -- bounds. These are used to limit the range of values that the optimizer can search (by varying the design variables) to find the optimum. Refer [discussion forum](https://discussions.udacity.com/t/model-constraints-vs-state-update-equations/250569) for details.

---

### PID Vs MPC

The PID approach would be analogous to a driver negotiating the road by continuously adjusting the input parameters (steering and throttle), correcting deviation from the reference trajectory, proceeding along as the new corners or obstacles appear in front. 
The MPC strategy would be analogous to studying the whole road, selecting the driving strategy before the departure and re-assessing/selecting the driving strategy at every time-step. Note that even the MPC approach does not guarantee 100% success as the strategy might have to be adjusted to changing conditions like rain, other road users, and so on.

Source: [openi.nlm.nih.gov](https://openi.nlm.nih.gov/detailedresult.php?img=PMC2784347_cc8023-1&req=4)

---

## Implementation

### Way-points transformation
The simulator provides a feed of values (in car coordinate system) containing the position of the car, its speed and heading direction. Additionally it provides waypoints (in map or global coordinate system) along a reference trajectory that the car is to follow. 
The coordinates of waypoints in vehicle coordinates are obtained by first shifting the origin to the current position of the vehicle and then, a 2D rotation to align with x-axis as the heading direction. This transformation results in vehicle's co-ordinates and yaw angle as zero. Thus, state of the car in the vehicle cordinate system is, state = [0, 0, 0, v, cte, epsi]

### Latency
There will be a delay as actuator command propagates through the car system. Realistic delay might be on the order of 100 milliseconds.
This problem is solved by running a simulation using the vehicle model starting from the current state for the duration of the latency (100 milliseconds). Resulting state from the simulation is the new initial state for MPC. Thus optimal trajectory is computed starting from the time after the latency period. Advantage is that the dynamics during the latency period is still calculated according to the vehicle model.

### Prediction Horizon
Optimization problem is formulated over a finite window of time starting from the current instant. Horizon keeps moving at every time-step. [t, t + T] to [t + dt, t + T + dt]
If N is the number of time-steps, dt is the duration of each time-step, prediction horizon is calculated as N * dt.
Short prediction horizons lead to more responsive controlers, but are less accurate and can suffer from instabilities when chosen too short. Long prediction horizons generally lead to smoother controls.
After a few trials, chose N = 20 and dt = 0.2

### Adjust weights on cost terms
Higher weights are added to cross-track error and orientation error to prioritize these cost terms.

During trials, Cost terms for absolute actuator values had minimal effect and car still manages to run fine high speeds. Hence, these terms are commented out.

### Velocity unit conversion from mph to mps
Tried converting velocity to mps units. Car loses stability, starts taking a zig-zag path and finally crashes. Hence, unit conversion is excluded.

---

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

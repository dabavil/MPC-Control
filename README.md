# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model Predictive Control Project


### Description of the model, incl. the state, actuators and update equations.

This controller uses a kinetic bicycle model. The model state contains 6 variables:

`[x, y, psi, v, cte, epsi]` => these represent vehicles x and y position, heading, velocity, cross-track-error, and orientation error respectively.

The above is a simplification vs a true dynamic model, in that it ignores effects like torque, tire / suspension dynamics, inertia, etc., but worked well for the task at hand and was much simpler to implement. 

The update from time 't' to 't+1' happens for the state variables via the following equations. (Variables at time 't', unless indicated otherwise)

`x[t+1] = x + v * cos(psi) * dt` // where dt is delta t

`y[t+1] = y + v * sin(psi) * dt`

`psi[t+1] = psi + v / Lf * delta * dt` // where delta is steering angle and Lf the constant reflecting distance between car's front and its center of gravity (set empirically)

`v[t+1] = v + a * dt` // where a is acceleration (throttle)

`cte[t+1] = f(x) - y + v * sin(epsi) * dt`

`epsi[t+1] = psi - psiDes + v * delta / Lf * dt` //where psiDes is the desired orientation at time t

The model uses two actuators - the steering angle, and the throttle / break. It also sets constraints on these controls to reflect car's physical limits: `[-25, +25]` degrees for the steering angle and `[-1, +1]` for the throttle (with -1 representing full breaking)

### Reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values, and previous values tried.

In my submission I have settled for N=10 steps and dt=0.1 seconds between steps. 

The values were chosen empirically, after trying combinations in range [5,20] for N, and [0.05 - 0.5] for dt. 
At the higher values of N, the model becomes computiationally intensive and led to errors / significant lag on my hardware setup. For the smaller N values, the model did not "see" far enough into the future to account for sharp turns, and kept leading the vehicle off track. 

For dt, lower values also led to computational intensity and did not provide better results, and for values above 0.2 the model was not responsive enough to react to changing road situation. In the end, combination of N=15 and dt=0.1 led to stable result, considering 1.5 seconds of future path, with the added benefit of dt aligning perfectly to the artificial 0.1s latency that is baked into the model (discussed below).


### Preprocessesing waypoints

Before running the MPC procedure, the major pre-processing step included transforming waypoints from the map coordinates into vehicle coordinates, which substantially simplified all the subsequent calculations. This involved subtracting the vehicle x and y from the waypoints to place the vehicle at the origin, and rotating them as to reflect zero orientation `psi`. The mathematical formulas are on lines 104-122 in the file `main.cpp`.


### Dealing with latency 

The file 'main.cpp' is built to send control inputs to the simulator with latency of 100ms, on top of any actual system latency. I have dealt with this by projecting the vehicle state forward by the latency duration, and feeding this anticipated state to the MPC controler, instead of the actual state at 't' that could be used if no latency was involved. 


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

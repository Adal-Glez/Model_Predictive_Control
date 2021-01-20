# **CarND-Controls-MPC Proejct**
### Adalberto Gonzalez

Self-Driving Car Engineer Nanodegree Program

Objective: Implement a non linear model Model Predictive Control to drive the car around the track. 

[image1]: ./img/result.gif

[video1]: ./img/result.mov

[![Link to my video result](/img/result.gif)](https://github.com/adl-aleb/Model_Predictive_Control/blob/master/img/result.mov)


## The Model

In this project, I've implemented a kinematic model to control the vehicle around the track. Kinematic models like this one ignore inertia, friction, gravity, torque and mass. This simplification reduces the accuracy of the models, but it also makes them more easy to manage. 
In this project, the vehicle is able to achieve the speed of 100 MPH.


--

The model will optimize the actuators input to simulate the vehicle trajactory and minimize the cost like cross-track error.

 
The state vector consists of [x, y, ψ, ν, δ, a]

x: cars x global position
y: cars y global position
ψ (psi): vehicle's angle in radians from the x-direction (radians)
ν : vehicle's velocity

including the actuators:
δ (delta): steering angle
a : acceleration (throttle)

and the equations:

*  x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt  // vehicle position on x
*  y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt  // vehicle position on y
*  psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt  // heading direction
*  v_[t+1] = v[t] + a[t] * dt  //velocity
*  cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt // Cross track error
*  epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt // Orientation error where Lf is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability.

## Timestep Length and Elapsed Duration (N & dt)

* N is number of timesteps.
* dt is the duration. 

having a large N could look like  an option however, large N will lead to a a higger computation cost and to a large cost function, in summary if I increase N the car would go off track and for dt the smaller dt could lead also to a higer computaion and response time could be affected.
so this is going 1.5 seconds into the future ( N =15 * dt = 0.1)



## Polynomial Fitting and MPC Preprocessing
prior to the polynomial fitting we transform the shift the car reference ( rotation about the origin )

substractt all points from current position  ( x and y coordinates will be at 0 )

double shift_x = ptsx[i]-px;
double shift_y = ptsy[i]-py;

rotate all of our points about the origin ( make size zero )

ptsx[i]=(shift_x * cos(0-psi) - shift_y*sin(0-psi));

ptsy[i]=(shift_x * sin(0-psi) + shift_y*cos(0-psi));

thay way our reference has 0 degrees also set to the origin, 
so when the car gets in the same orientation fo the line we end up with an horizontal line which is easier to follow as a function with simpler values in the x 
Ive found out that is also helpfun with the mpc validation and to do crosstrack error
              

## Model Predictive Control with Latency

For our intial state model, our algorithm selects an optimal sequence of steering and throttle adjustments, 150 times a second for the time forward.

We account for latency by assuming the current car drifts at the current speed, heading, and rate of turn for the entire interval forward.

This is a similar excersice to forsee the next action while driving, since you have to take care of the steps ahead more than your current position that is pretty much fixed and you cannot change it unless you advance.




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
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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

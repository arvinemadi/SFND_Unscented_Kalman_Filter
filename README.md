# SFND Unscented Kalman Filter - COMPLETED PROJECT 
Sensor Fusion UKF Highway Project

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project an Unscented Kalman Filter is implemented to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project required obtaining RMSE values that were lower than the tolerance specifications. 



The base and data from Udacity. The programs in src/ukf.cpp, and src/ukf.h have been completed. The program main.cpp has several visualization features that can be modified.

<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

Step taken:
- Initializing the state variable and covariance matrix. CTRV (constant turn rate and velocity magnitude model has been used)
- Prediction:
-- Augmented sigma points are created for the current state and covariance matrix
-- New sigma points are predicted using the time lapsed from the last measurement
-- Using the new sigma points, new state and covariance matrix are predicted
- Measurement update depending if the measurement was from Radar or Lidar but the process steps are the same
-- All sigma points are mapped into measurement space (for Radar, r, phi, and rate change of r and for Lidar x, and y locations)
-- Mean and covariance matrix of the mapped sigma points are calculated. Since measurement noise is independent, measurement noise covariance matrix R is added to S.
-- Cross-correlation between predicted sigma points in the state space and predicted sigma points in measurement space are used together with covariance matrix of the measured noise to find K (Kalman gain)
-- Using the new measured data, the new state is updated with a new time stamp

## Reference:
- The unscented Kalman Filter for Nonlinear Estimation: https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
- UKF tutorial: https://www.cse.sc.edu/~terejanu/files/tutorialUKF.pdf
- ROS implements EKF and UKF. Well documented and can be used to fuse different sensors: http://wiki.ros.org/robot_localization
---
The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment
and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.


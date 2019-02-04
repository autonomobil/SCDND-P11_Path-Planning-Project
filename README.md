[//]: # (Image References)

[img1]: ./images/waypoints1.png "waypoints1"
[img2]: ./images/waypoints2.png "waypoints2"
[img3]: ./images/state.png "state"
[img4]: ./images/example.png "example"
___
# SDCND Term 3 Project 11: Path Planning
## Path Planning Project for Udacity's Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit. The path planner is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

#### The results can be viewed here([Youtube](https://youtu.be/8EBtGReGX-E)):
[![result1](https://img.youtube.com/vi/8EBtGReGX-E/0.jpg)](https://youtu.be/8EBtGReGX-E)


## Overview
The highway track, which is characterized by the given waypoints looks like this in the global coordinate system (dots = waypoints):
![img1]

A closer look reveals that a linear interpolation between the sparse waypoints leads to sudden changes in direction and speed, so the limits of acceleration and jerk are broken.
![img2]
That's the reason why the included library spline.h is used [(link)](https://kluge.in-chemnitz.de/opensource/spline/) to smooth out the trajectories. Another option would be to use Jerk-Minimizing-Trajectories, but the use of spline interpolation gave sufficient results.



## Detailed
### Sensor Fusion & Prediction [line 113 to line 179](./src/main.cpp#L113)
This part of the code deal with the telemetry and sensor fusion data. It intents to reason about the environment. The following ego car's localization data can be obtained from the simulation:
+ car_x
+ car_y
+ car_s
+ car_d
+ car_yaw
+ car_speed 
+ previous_path_x 
+ previous_path_y  In the case, we want to know three aspects of it:

- Is there a car in front of us blocking the traffic.
- Is there a car to the right of us making a lane change not safe.
- Is there a car to the left of us making a lane change not safe.

These questions are answered by calculating the lane each other car is and the position it will be at the end of the last plan trajectory. A car is considered "dangerous" when its distance to our car is less than 30 meters in front or behind us.

### Behaviour Planning [line 180 to line 229](./src/main.cpp#L180)
This part decides what to do:
  - If we have a car in front of us, do we change lanes?
  - Do we speed up or slow down?

Based on the prediction of the situation we are in, this code increases the speed, decrease speed, or make a lane change when it is safe. Instead of increasing the speed at this part of the code, a `speed_diff` is created to be used for speed changes when generating the trajectory in the last part of the code. This approach makes the car more responsive acting faster to changing situations like a car in front of it trying to apply breaks to cause a collision.

### Trajectory Generation [line 230 to line 341](./src/main.cpp#L230)
This code does the calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points.

First, the last two points of the previous trajectory (or the car position if there are no previous trajectory, lines 321 to 345) are used in conjunction three points at a far distance (lines 348 to 350) to initialize the spline calculation (line 370 and 371). To make the work less complicated to the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates (lines 361 to 367).

In order to ensure more continuity on the trajectory (in addition to adding the last two point of the pass trajectory to the spline adjustment), the pass trajectory points are copied to the new trajectory (lines 374 to 379). The rest of the points are calculated by evaluating the spline and transforming the output coordinates to not local coordinates (lines 388 to 407). Worth noticing the change in the velocity of the car from line 393 to 398. The speed change is decided on the behavior part of the code, but it is used in that part to increase/decrease speed on every trajectory points instead of doing it for the complete trajectory.

#### Sensor Fusion & Prediction

![img3]

#### Behaviour Planning






## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator

## Setup and Running
These are the suggested steps for **Windows** setup:
* [Original repository](https://github.com/udacity/CarND-MPC-Project)
* Follow these [instructions](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up Ubuntu BASH.
* Download Windows simulator [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).
* Open Ubuntu Bash (write following commands to Ubuntu Bash command window)
* ``sudo apt-get update``
* ``sudo apt-get install git``
* ``sudo apt-get install cmake``
* ``sudo apt-get install openssl``
* ``sudo apt-get install libssl-dev``
* navigate to where you want to clone this repository to, for example:
 ``cd /mnt/c/Users/Bob``
* ``git clone https://github.com/autonomobil/SCDND-P11_Path-Planning-Project``
* navigate to project folder: ``cd SCDND-P11_Path-Planning-Project``
* ``sudo rm /usr/lib/libuWS.so``
* ``./install-ubuntu.sh``
* navigate to the build folder: ``cd build``
* Execute ``cmake .. && make``
* Launch the **term3_sim.exe** from Windows simulator folder
* Execute ``./path_planning``
* If you see ``Listening to port 4567 Connected!!!``, it is working
* Press **Select**




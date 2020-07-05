# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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
## Project Approch to Path Planning

In the implemented solution, there were 2 key portions of the logic. The first was the behavioural planning portion and the second was the path generation. The following sections will go through both of these sections.

### Behavioural Planning

The behavioural planning portion was responsible for understanding and predicting the behaviour of vehicles around our vehicle and using this information to decide when to slow down, speed up or change lanes. The sensor fusion information provided details of all the vehicles, their positions and velocities and their ID. At the core of the algorithm,  the logic determined when a vehicle was an obstacle in front our vehicle, and then used neighboring vehicle information to determine when to change lanes. 

#### Obstacle detection and Lane Change Logic

The logic to determine when there was a vehicle acting as an obstacle infront of us was implemented as follows: 
> **IF** there is a vehicle in the same lane as me, ahead of me and the relative distance is less than a distance threshold calculated by a calibratable time multipled by the relative velocity of the two vehicles **OR IF** The vehicle is ahead of me but within a distance our vehicle will reach within a calibratable time, the vehicle ahead is determined to be an obstacle.

When we have determined that a vehicle ahead is an obstacle, we also adjust our desired velocity to that of the vehicle ahead so we follow at the velocity of the leader vehicle and reduce probability of collision.

The lane change logic was implemted such that we set lane change as available, unless there is a vehicle in the neighboring lane preventing a safe lane change from occuring:
> **IF** the desired lane is valid and the sensed vehicle is in the desired lane, and the relative velocity of the sensed vehicle would bring it within a danger threshold of distance to our vehicle, **OR IF** the desired lane is valid and the sensed vehicle is in the desired lane, and the position of the sensed vehicle would bring it too close to our vehicle based on our velocity, the prevent lane change to the desired lane. 


After checking for the obstacles and if the left and right lane change is available, we then make check to see which lane is moving the fastest and then make decisions based on the calculated threats and options.

First, we only act if an obstacle is in the way, otherwise we just continue driving in the current lane at the speed limit. However, if an obstacle is detected, we can then slow down, or change lanes. In order of preference, we first desire to change lanes, and then slow down. The preference is to first change lanes, where we first check the left lane, then the right lane and then slow down. The speed we slow down to is dictated by the speed of the sensed obstacle.

Once we decide on what we want to do: continue at the speed limit in the same lane, change lanes to the left, change lanes to the right or slow down to track the slower vehicle aheadl, we generate the path to actuate the decision. 

#### Path Generation

For the path generation, we use the spline tool suggested for the project to generate the path. We start out by taking the last 2 points of the vehicle to anchor the spline and then we add 3 new points to generate a smooth spline that allows continuity in terms of vehicle path.The 3 new points are generated using map waypoints based on the vehicles current position and where we anticpate it will be at 3 look ahead times in the future based on the vehicles maximum velocity.

We then take the spline and the previous path to send the next points to the controller. We start with all the previous points and then add additional points to send a total of 50 points to the controller. We break down the points such that the x-points are spaced out based on our sample time to track the desired longitudinal velocity. 

## Observations and Conclusion

The implementation worked quite well. In my trial, it managed to cover two complete loops of the track without incident and while mainting velocity, acceleration and jerk conditions. It does however have quite a simple behavioural planner and this allows for room for improvement. I believe implementing a cost function implementation considering not only current obstacles but future obstacles and then changing lanes to optimize the drive on the highway would result in an optimal solution. Currently, the implementation can result in a poor decision being made in terms of a lane change where it get stuck after crossing the immediate obstacle because it changed to the wrong lane. The implementation of a cost function would resolve this. I believe there is room to implement a more robust polynomial path as well which can directly consume acceleration and jerk constraints in the calulcation of the polynomial. 



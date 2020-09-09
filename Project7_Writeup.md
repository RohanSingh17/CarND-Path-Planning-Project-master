## Project: Highway Driving
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
In this project we learned about how to plan a path (or trajectory) for a self driving car
while driving on a highway to navigate through traffic.
The project in the module simulates a vehicle driving on a highway using a 
simulator where road traffic (consisting of cars) is generated randomly. The 
objective was to navigate through traffic for a certain distance without causing
any incidents which might be considered fatal while driving an actual car on the road.
These include collision with other cars, driving in the middle of the lane, excessive
jerk, overspeeding etc. 

### Path Generation
The approach to path generation is kept simple for this code. The movement of 
car is restricted to its own lane until another car driving at slower speed in 
the same lane is encountered. When such a situation occurs, the car starts tracking 
its distance from the car in front and if it goes below a certain predefined value
(30 units) the car starts decelerating by a constant rate. Simultaneously, 
the algorithm searches for an empty space in the adjacent lane. If an empty space 
is found within suitable distance (i.e. 30 m ahead and 10m behind). The algorithm 
considers a lane change which is prioritized based on maximum length of unoccupied 
portion in the adjacent lanes ahead of the car.

### Code Reference
The code reference is taken from the file main.cpp which contains the modified code.

The variable defined below are used to define the current lane, current speed and 
target speed which is defined as 49.0 mph (lines 17-20 from main.cpp)

    int curr_lane=1;
    double target_speed=49.0;
    double curr_speed=0;

The points _`ptsx, ptsy`_ contain anchor points for trajectory generation. The code 
from lines 118-144 are used to decide intial points for trajectory generation.

    vector<double> ptsx;	
    vector<double> ptsy;

The next lines shown below are used to define current car_d which assigns current
value to a variable `curr_lane`.

    if ((car_d>0) && (car_d<4))
      curr_lane=0;
    else if ((car_d>4) && (car_d<8))
      curr_lane=1;
    else
      curr_lane=2;
      
Our car constantly tracks its distance from other cars ahead in front of it in its 
lane and if the distance is below `dist_car` . The car starts decelerating until
its distance increases. The other variables `min_dist_lane` stores minimum distances 
from the cars in front of our car in all the lanes and `lane_closest cars` stores ids of 
closest cars in front and rear of the our car in all the lanes

    double dist_car=30.0;     
    vector<double> min_dist_lane(3,99999.0);  
    vector<int> lane_closest_cars(6,-1);

The code from lines 170-204 loops through all the cars stored in `sensor_fusion` 
variable and checks for closest cars in front of the our car in all lanes and stores
minimum distances in variable `min_dist_lane` and closest car ids in front are stored 
as first 3 elements of `lane_closest_cars`. If the other car is behind the our car by a distance
not greater than 10 units its id is stored last 3 elements of `lane_closest_cars` for 
respective lanes otherwise -1 is assigned in place which means there is no car in close 
range behind our car.

    int future_lane=curr_lane;

The variable `future_lane` is declared and its value is decided based on codes in lines 
210-232. Initially code checks if minimum distance in `curr_lane` is less than `dist_car`
(30 units). If it is less than the required distance our car starts decelerating. It 
then checks whether our car is in middle lane or not. If it is in middle lane if 
subsequently checks for 3 three things to decide whether to go in left or right lane.
 
1) Which car lane has larger minimum distance ahead.
2) If the minimum distance in adjacent lane is greater minimum distance in current lane.
3) If there are no cars next to or behind it in the adjacent lane.

and if our car is in side lanes it check for 2. and 3. points shown above only to make 
decision for lane change.

In case the distance from the car in front is not less than minimum required distance, 
cars speed is increased if it is found to be less than target speed.

The `future_lane` variable is the used define anchor points ahead in the future lane.
The points once decided a spline is generated using anchor points and a smooth trajectory 
is formed for car to follow.

# Path Planning with Finite State Machines
This repository was created as a submission for the Path Planning project of Udacity's Self Driving Car Nanodegree. The goal of the Path Planning project was to create a path planner in C++ and test it against Udacity's simulator.


## Overview
The main objective of this project was to create a C++ program which communicates with a highway simulator, sending path coordinates to drive the car under a given speed, avoid collisions and make lane changes if necessary. The project was completed using finite state machines and cost functions as described below.

## Inputs

The inputs from the simulator to the path planner are the following:
- Main car's localization data: position in map/Frenet coordinates, yaw angle, speed
- Previous path data: list of previous points sent by the planner, with the processed points removed
- Previous path's end s and d values (Frenet coordinates)
- Sensor fusion data: a 2D vector of all other cars' attributes on the same side of the road (position, speed, ID)

## Outputs

The path planner's output to the simulator is a list of x-y coordinates for the vehicle to visit every 0.02 seconds.

## Implementation of the path planner

The path planner is implemented mainly in the ```Planner``` class (in ```planner.cpp```), which contains the most important functions:
- ```get_successor_states()```
- ```set_speed()```
- ```generate_trajectory()```,

as well as the cost functions used for the state transition function.

The state transition function (used for selecting the next state based on possible next states and their associated costs) is implemented in ```main.cpp``` (lines 106 - 128).

Additionally, the ```Vehicle``` class is used to store vehicle data, such as the current position, lane, speed, and state of the ego vehicle.

### Finite state machines

Implementation of the path planner relies heavily on the concept of finite state machines and a very simplified model of highway driving. In this model, the vehicle has 3 states:
- lane keeping
- changing lanes to the left
- changing lanes to the right

### Cost functions
## Running the code

## Pass criteria for the project

## Results and Summary



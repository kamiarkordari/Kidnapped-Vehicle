Implementing a 2 dimensional particle filter to localize a self-driving car using a map, GPS, lidar, and radar measurements.

## Overview
Our robot (an autonomous vehicle) has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project we will implement a 2 dimensional particle filter in C++. Particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

## Background
One of the important modules in a self driving vehicle is the localization module. Localization is estimating the location of the vehicle with high accuracy (3-10 cm) in reference to a global map.

One way to localize a vehicle is by using data from Global Positioning System (GPS). But GPS doesn't provide high accuracy data. In the best case the accuracy of GPS is 1-3 m. In cases where GPS signal is weak, the accuracy drops to 50-100 m.

To achieve an accuracy of 3-10 cm, sensor information from LIDAR and/or RADAR or INU is used by applying a Particle Filter.

## Localization Algorithm
Localization in context of self driving vehicle makes use of GPS, LIDAR and/or RADAR, and a map of landmarks based on the following algorithm:

1. A global map of landmarks is constructed for places where the self driving vehicle is to be deployed. This map contains information about different 'landmarks'. Landmarks are features that can be uniquely identified and are not subject to change for a long period of time. These landmarks are used to estimate the relative location of car to them.

2. Once a map is constructed, GPS on the vehicle is used to initialize the vehicle and select a small portion of global map around the vehicle to reduce the amount of computation as the algorithms must run in real time.

3. GPS provides noisy measurement and is not enough for accurate localization. LIDAR and/or RADAR sensor on the vehicle is used for map matching by measuring the distance between the vehicle and the map landmarks around the vehicle. This helps in further pinning down the location of the vehicle as it is now relative to landmarks in the global map. However, LIDAR and RADAR information is also not accurate and prone to noise.

4. Particle Filter is used to combine the information gained from all above steps and predict the location of car with high accuracy of 3-10 cm.

The whole algorithm repeats at run time when the car is moving and new location of car is predicted.

## Particle Filter

The flowchart below represents the steps of the particle filter algorithm as well as its inputs.

![Particle Filter](media/particle_filter_flowchart.png)

The four main steps in the algorithm are:
1. **Initialization**: At this step we create a set of particles and initialize their locations by using the GPS measurement. I picked 1000 particles. The subsequent steps in the process will refine this estimate to localize the vehicle with every new observation and input. This is implemented as `init` method of the `ParticleFilter`class.
2. **Prediction**: At this step we predict where the vehicle will be at the next time step, by updating particles based on yaw rate and velocity, while accounting for Gaussian sensor noise. This is implemented as `prediction` method of the `ParticleFilter`class.
3. **Weight update**: At this step we update particle weights using map landmark positions and feature measurements. This is implemented as `updateWeights` method of the `ParticleFilter`class.
4. **Resampling**: At this step we generate a new set of particles by resampling using particle weights. Resampling is performed by randomly drawing new particles from old ones with replacement in proportion to their importance weights. The new set of particles represents the Bayes filter posterior probability. This gives us a refined estimate of the vehicles position based on input evidence. This is implemented as `resample` method of the `ParticleFilter`class.

#### Weight Update
In the weight update step we need to perform observation measurement transformations, along with identifying measurement landmark associations in order to correctly calculate each particle's weight.

**Transformation:** We first need to transform the car's measurements from its local car coordinate system to the map's coordinate system. Since we know the coordinates of the particle from the car's frame of reference we can use this information and a matrix rotation/translation to transform each observation from the car frame of reference to the map frame of reference. This is implemented as `transformedCoords` method of the `ParticleFilter`class.   

**Association:** Next, each landmark measurement will need to be associated with a landmark identifier that represents a real world object. We associate the closest landmark to each transformed observation. This is implemented as `dataAssociation` method of the `ParticleFilter`class.

**Calculate Weight:** Finally, we calculate the weight value of the particle. The particles final weight will be calculated as the product of each measurement's Multivariate-Gaussian probability density. This is implemented as `calculateWeights` method of the `ParticleFilter`class. We calculate each measurement's Multivariate-Gaussian probability density using the below code:

```C++
double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
double sigma_x = std_landmark[0];
double sigma_y = std_landmark[1];

double exponent = (pow(obs.x - best_landmark.x, 2) / (2 * pow(sigma_x, 2)))
               + (pow(obs.y - best_landmark.y, 2) / (2 * pow(sigma_y, 2)));

double weight = gauss_norm * exp(-exponent);
```

## The code
### Running the Code
This project involves a Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh


Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


### Repo Structure
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

`particle_filter.cpp` in the `src` directory contains the scaffolding of a `ParticleFilter` class and some associated methods.

`src/main.cpp` contains the code that runs the particle filter by calling the associated methods.

### Inputs to the Particle Filter
Inputs to the particle filter are in the `data` directory. `map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. This Map data is provided by 3D Mapping Solutions GmbH.

All other data the simulator provides, such as observations and controls.


## Demo

Below is a video of the a demo of this algorithm. The particle filter processes input data to calculate the real-time estimation of the vehicle’s location and heading orientation. This location is shown with a blue circle (with a black arrow inside for the heading).

The blue car shows the ground truth (the actual position and heading orientation of the car). It is visualized only for comparison purpose.


##### Success Criteria
To check the particle filter's performance we can run ./run.sh in the terminal. If it meets the specifications, there will be a "Success! Your particle filter passed!" message.

These are the two performance metrics:

1. **Accuracy**: The particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: The particle filter should complete execution within the time of 100 seconds.


![Particle Filter Demo](media/kidnapped_vehicle.gif)

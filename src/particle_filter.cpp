/**
 * particle_filter.cpp
 *
 * Created on: Feb 29, 2020
 * Author: Kamiar Kordari
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * Add random Gaussian noise to each particle.
   */
  num_particles = 100;

  normal_distribution<double> N_x(x, std[0]);
  normal_distribution<double> N_y(y, std[1]);
  normal_distribution<double> N_theta(theta, std[2]);
  std::default_random_engine gen;

  for (int i = 0; i < num_particles; i++)
  {
    Particle particle;
    particle.id = i;
    particle.x = N_x(gen);
    particle.y = N_y(gen);
    particle.theta = N_theta(gen);
    particle.weight = 1;

    particles.push_back(particle);
    weights.push_back(1);

  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   */

   std::default_random_engine gen;

   double new_x;
   double new_y;
   double new_theta;

   for (int i = 0; i < num_particles; i++)
   {
     if (fabs(yaw_rate)< 0.0001)
     {
       new_theta = particles[i].theta;
       new_x = particles[i].x + velocity * delta_t * cos(new_theta);
       new_y = particles[i].y + velocity * delta_t * sin(new_theta);
     }
     else
     {
       new_theta = particles[i].theta + yaw_rate * delta_t;
       new_x = particles[i].x + velocity / yaw_rate * (sin(new_theta) - sin(particles[i].theta));
       new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(new_theta));
     }

     normal_distribution<double> N_x(new_x, std_pos[0]);
     normal_distribution<double> N_y(new_y, std_pos[1]);
     normal_distribution<double> N_theta(new_theta, std_pos[2]);

     particles[i].x = N_x(gen);
     particles[i].y = N_y(gen);
     particles[i].theta = N_theta(gen);
   }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * Update weights of particles
   */

   LandmarkObs converted_obs;
   LandmarkObs best_landmark;

   weights.clear();

   for (int i = 0; i < int(particles.size()); i++) {
     double prob = 1.0;

     for (int j = 0; j < int(observations.size()); j++) {
       //  Convert observation to map coordinate
       converted_obs = transformCoords(particles[i], observations[j]);

       //  Associate observation to a landmark
       best_landmark = dataAssociation(converted_obs, map_landmarks, std_landmark);

       //  Calculate weight
       double w = calculateWeights(converted_obs, best_landmark, std_landmark);

       //  Get final weight by multiplying all the probabilities
       prob *= w;
     }

     particles[i].weight = prob;
     weights.push_back(prob);
  }
}


LandmarkObs ParticleFilter::transformCoords(Particle p, LandmarkObs obs) {
  /**
   * Convert coordinates from particle to map
   */
  LandmarkObs transformed_coords;

  transformed_coords.id = obs.id;
  transformed_coords.x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
  transformed_coords.y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;

  return transformed_coords;
}


LandmarkObs ParticleFilter::dataAssociation(LandmarkObs converted_obs, Map map_landmarks, double std_landmark[]) {

  LandmarkObs best_landmark;

  for (int i = 0; i < int(map_landmarks.landmark_list.size()); i++) {
    double min_dist;
    double distance = dist(converted_obs.x, converted_obs.y, map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f);
    if (i == 0) {
      min_dist = distance;
      best_landmark.id = map_landmarks.landmark_list[i].id_i;
      best_landmark.x = map_landmarks.landmark_list[i].x_f;
      best_landmark.y = map_landmarks.landmark_list[i].y_f;
    }
    else if (distance < min_dist)
    {
      min_dist = distance;
      best_landmark.id = map_landmarks.landmark_list[i].id_i;
      best_landmark.x = map_landmarks.landmark_list[i].x_f;
      best_landmark.y = map_landmarks.landmark_list[i].y_f;
    }
  }
  return best_landmark;
}


double ParticleFilter::calculateWeights(LandmarkObs obs, LandmarkObs best_landmark, double std_landmark[]) {
  /**
   * Calculate the weight value of the particle
   */

  double sigma_x = std_landmark[0];
  double sigma_y = std_landmark[1];
  double gauss_norm = 1 / (2 * M_PI * sigma_x * sigma_y);

  double exponent = (pow(obs.x - best_landmark.x, 2) / (2 * pow(sigma_x, 2)))
                 + (pow(obs.y - best_landmark.y, 2) / (2 * pow(sigma_y, 2)));

  double weight = gauss_norm * exp(-exponent);

  return weight;
}



void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional to their weight
   */
   std::default_random_engine gen;
   std::discrete_distribution<int> distribution(weights.begin(), weights.end());

   vector<Particle> resample_particles;

   for (int i = 0; i < num_particles; i++)
   {
     resample_particles.push_back(particles[distribution(gen)]);
   }

   particles = resample_particles;

}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

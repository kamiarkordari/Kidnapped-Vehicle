/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  normal_distribution<double> N_x(x, std[0]);
  normal_distribution<double> N_y(y, std[1]);
  normal_distribution<double> N_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++)
  {
    particle particle;
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
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

   default_random_engine gen;

   for (int i = 0; i < num_particles; i++)
   {
     double new_x;
     double new_y;
     double new_theta;

     if (yaw_rate == 0)
     {
       new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
       new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
       new_theta = particles[i].theta;
     }
     else
     {
       new_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
       new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta  + yaw_rate * delta_t));
       new_theta = particles[i].theta + yaw_rate * delta_t;
     }

     normal_distribution<double> N_x(new_x, std_pos[0]);
     normal_distribution<double> N_y(new_y, std_pos[1]);
     normal_distribution<double> N_theta(new_theta, std_pos[2]);

     particles[i].x = N_x(gen);
     particles[i].y = N_y(gen);
     particles[i].theta = N_theta(gen);
   }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

   double weight_normalizer = 0.0;

   for (int i = 0; i < num_particles; i++)
   {
     double particle_x = particles[i].x;
     double particle_y = particles[i].y;
     double particle_theta = particles[i].theta;

     // transformation observatios from vehicle to map coordinates
     vector<LandmarkObs> transformed_observations;

     for (int j = 0; j < observations.size(); j++)
     {
       LandmarkObs transformed_obs;
       obs = observations[j];

       transformed_obs.x = particle_x + (cos(particle_theta) * obs.x) - (sin(particle_theta) * obs.y);
       transformed_obs.y = particle_y + (sin(particle_theta) * obs.x) + (cos(particle_theta) * obs.y);
       transformed_observations.push_back(transformed_obs);
     }


     // Pick landmarks based on proximity
     vector<LandmarkObs> nearny_landmarks;
     for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
     {
       Map::single_landmark_s current_landmark = map_landmarks.landmark_list[j];
       dist_x = particle_x - current_landmark.landmark_x_f;
       dist_y = particle_y - current_landmark.landmark_y_f;
       if sqrt((dist_x * dist_x) + (dist_y * dist_y)) <= sensor_range {
         nearny_landmarks.push_back(LandmarkObs {current_landmark, current_landmark.x_f, current_landmark.y_f});
       }
     }

     dataAssociation(nearny_landmarks, transformed_observations, sensor_range);

     particles[i].weight = 1.0;

     // Calculate weight
     double sigma_x = std_landmark[0];
     double sigma_y = std_landmark[1];
     double sigma_x_2 = sigma_x * sigma_x;
     double sigma_y_2 = sigma_y * sigma_y;
     double normalizer = 1.0 / (2 * M_PI * sigma_x * sigma_y);

     for (int j = 0; j < transformed_observations.size(); j++){
       double trans_obs_x = transformed_observations[j].x;
       double trans_obs_y = transformed_observations[j].y;
       double trans_obs_id = transformed_observations[i].id;

       double multi_prob = 1.0;

       for (int l = 0; l < nearny_landmarks.size(); l++){
         double pred_landmark_x = nearny_landmarks[l].x;
         double pred_landmark_y = nearny_landmarks[l].y;
         double pred_landmark_id = nearny_landmarks[l].id;

         if (trans_obs_id == pred_landmark_id){
           multi_prob = normalizer * exp(-1.0 * ((pow((trans_obs_x - pred_landmark_x), 2)/(2.0 * sigma_x_2)) + (pow((trans_obs_y - pred_landmark_y), 2)/(2.0 * sigma_y_2))));
           particles[i].weight *= multi_prob;
         }
       }
     }
     weight_normalizer = particles[i].weight;
   }

   // Normalize weight_sum
   for (int i = 0; i < particles.size(); i++){
     particles[i].weight /= weight_normalizer;
     weights[i] = particles[i].weight;
   }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   default_random_engine gen;
   discrete_distribution<int> distribution(weights.begine(), weights.end());

   vector<Particle> resample_particles;

   for (int i = 0; i < num_particles; i++)
   {
     resmaple_particles.push_back(particles[distribution(gen)]);
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

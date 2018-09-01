/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// random engine for gaussian
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    // number of particles
    num_particles = 100;
    
    // normal distribution for x, y and theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    // initaialize particles
    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        
        // append to 'particles'
        particles.push_back(p);
    }
    
    // set is_initilalized to true
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // gausiann noise, mean = 0
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    
    // predict next state
    for(int i=0; i<num_particles; i++){
        if (fabs(yaw_rate) < 0.00001){
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        } else {
            particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
            particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }
        
        // add gaussian noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
    
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    
    for (int i = 0; i < observations.size(); i++){
        LandmarkObs obs = observations[i];
        
        // init min dist with max. available value
        double min_dist = numeric_limits<double>::max();
        
        // init map id of the closest prediction with -1
        int map_id = -1;
        
        for (int j = 0; j < predicted.size(); j++) {
            LandmarkObs pred = predicted[j];
            
            // distance between pred and obs
            double dist_obs_pred = dist(obs.x, obs.y, pred.x, pred.y);
            
            // find the predicted landmark closest the current observed landmark
            if (dist_obs_pred < min_dist) {
                min_dist = dist_obs_pred;
                map_id = pred.id;
            }
            
        }
        // set the observation's id as the id of closest predictied landmark
        observations[i].id = map_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    
    
    for(int i=0; i<num_particles; i++){
        
        // particle x, y and theta
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;
        
        // vector of map landmarks those in sensor range from the particle
        vector<LandmarkObs> predictions;
        
        for(int j=0; j<map_landmarks.landmark_list.size(); j++){
            
            float lm_x = map_landmarks.landmark_list[j].x_f;
            float lm_y = map_landmarks.landmark_list[j].y_f;
            int lm_id = map_landmarks.landmark_list[j].id_i;
            
            // distance between particle and landmark
            double dist_p_lm = dist(p_x, p_y, lm_x, lm_y);
            if(dist_p_lm <= sensor_range){
                // append landmark that in the sensor range
                predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
            }
            
        }
        
        // transform from car coordinates to map coordinates
        vector<LandmarkObs> transformed_obs;
        for (int j = 0; j < observations.size(); j++) {
            double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
            double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
            transformed_obs.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
        }
        
        // Find the predicted measurement that is closest to each observed measurement and assign the
        // observed measurement to this particular landmark.
        dataAssociation(predictions, transformed_obs);
        
        // update weight
        // reset weight with 1.0
        particles[i].weight = 1.0;
        
        for (int j = 0; j < transformed_obs.size(); j++){
            
            // get x, y, and the associated pred_id from transformed_obs
            double tobs_x = transformed_obs[j].x;
            double tobs_y = transformed_obs[j].y;
            int associated_pred_id = transformed_obs[j].id;
            
            // get the x,y from the prediction that has associated pred_id
            double pred_x, pred_y;
            for (int k = 0; k < predictions.size(); k++) {
                if (predictions[k].id == associated_pred_id) {
                    pred_x = predictions[k].x;
                    pred_y = predictions[k].y;
                }
            }
            
            // weight calc with multivariate Gaussian
            double std_x = std_landmark[0];
            double std_y = std_landmark[1];
            double tobs_pred_w = ( 1/(2*M_PI*std_x*std_y)) * exp( -( pow(pred_x-tobs_x,2)/(2*pow(std_x, 2)) + (pow(pred_y-tobs_y,2)/(2*pow(std_y, 2))) ) );
            
            // product weight
            particles[i].weight *= tobs_pred_w;
        }
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    vector<Particle> resampled_particles;
    
    // get all of the particles weight
    vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }
    
    //Generate random particle index
    uniform_int_distribution<int> particle_idx(0, num_particles - 1);
    int index = particle_idx(gen);
    
    // beta for resample wheel
    double beta = 0.0;
    
    // maximum weight
    double max_weight = *max_element(weights.begin(), weights.end());
    
    // uniform random distribution (0.0, 2 * max_weight)
    uniform_real_distribution<double> random_weight(0.0, (2.0 * max_weight));

    // resample wheel
    for (int i = 0; i < num_particles; i++){
        beta += random_weight(gen);
        while(weights[index] < beta){
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }
    particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

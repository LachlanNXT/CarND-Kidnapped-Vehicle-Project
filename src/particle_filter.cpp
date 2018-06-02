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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 10;
	default_random_engine gen;
	
	// This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_t(theta, std[2]);

	for (int i = 0; i < num_particles; i++)
	{
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y =  dist_y(gen);
		particle.theta = dist_t(gen);
		particle.weight = 1.0;
		particles.push_back(particle);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	for (int i = 0; i<num_particles; i++)
	{
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		if (fabs(yaw_rate) < 0.0001)
		{
			x = x + velocity*cos(theta)*delta_t;
			y = y + velocity*sin(theta)*delta_t;
		}
		else
		{
			x = x + velocity/yaw_rate*(sin(theta+yaw_rate*delta_t)-sin(theta));
			y = y + velocity/yaw_rate*(-cos(theta+yaw_rate*delta_t)+cos(theta));
			theta = theta + yaw_rate*delta_t;
		}
		normal_distribution<double> dist_x(x, std_pos[0]);
		normal_distribution<double> dist_y(y, std_pos[1]);
		normal_distribution<double> dist_t(theta, std_pos[2]);
		x = dist_x(gen);
		y = dist_y(gen);
		theta = dist_t(gen);
		particles[i].x = x;
		particles[i].y = y;
		//theta = atan2(sin(theta),cos(theta));
		particles[i].theta = theta;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i<observations.size(); i++)
	{
		double mindist = 999;
		int label = -1;
		for (int j = 0; j<predicted.size(); j++)
		{
			double test = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			//cout << "test " << test << " mindist " << mindist << " label " << label << " id " << predicted[j].id << endl;
			if (test < mindist)
			{
				mindist = test;
				label = j; //predicted[j].id;
			}
		}
		observations[i].id = label;
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
	//   http://planning.cs.uiuc.edu/node99.
	double max_weight = 0;
	for (int i = 0; i<num_particles; i++)
	{
		particles[i].weight = 0;
		// transform observations into the map frame relative to the particle
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		std::vector<LandmarkObs> transformed_obs = observations;
		//cout << "updating particle " << i << endl;
		for (int j = 0; j<observations.size(); j++)
		{
		// transform to map x coordinate
		transformed_obs[j].x = x + (cos(theta) * observations[j].x) - (sin(theta) * observations[j].y);
		// transform to map y coordinate
		transformed_obs[j].y = y + (sin(theta) * observations[j].x) + (cos(theta) * observations[j].y);
		}
		//cout << "particle " << i << " " << x << "," << y << "," <<  theta << "," << endl;
		//cout << " observation " << observations[0].x << "," << observations[0].y << endl;
		//cout << " transformed " << transformed_obs[0].x << "," << transformed_obs[0].y << endl;
		// find all landmarks that are within sensor range
		std::vector<LandmarkObs> predicted;
		for (int j = 0; j<map_landmarks.landmark_list.size(); j++)
		{
			double x1 = map_landmarks.landmark_list[j].x_f;
			double y1 = map_landmarks.landmark_list[j].y_f;
			if (dist(x,y,x1,y1) < sensor_range)
			{
				LandmarkObs landmark;
				landmark.id =map_landmarks.landmark_list[j].id_i;
				landmark.x = map_landmarks.landmark_list[j].x_f;
				landmark.y = map_landmarks.landmark_list[j].y_f;
				predicted.push_back(landmark);
				//cout << "landmark" << landmark.id << endl;
			}
		}

		// perform nearest neighbor association on predicted landmarks
		dataAssociation(predicted, transformed_obs);
		//cout << "predicted landmark " <<  transformed_obs[0].id << " - " << predicted[transformed_obs[0].id].x << "," << predicted[transformed_obs[0].id].y << endl;
		//cout << " transformed obs " << transformed_obs[0].x << "," << transformed_obs[0].y << endl;

		// guassian probability of each measurement
		for (int j= 0; j<transformed_obs.size(); j++)
		{
			if (transformed_obs[j].id > -1)
			{
				// calculate normalization term
				double gauss_norm= (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
				double x_diff = (transformed_obs[j].x - predicted[transformed_obs[j].id].x);
				double y_diff = (transformed_obs[j].y - predicted[transformed_obs[j].id].y);
				// calculate exponent
				double exponent= (x_diff*x_diff)/(2 * std_landmark[0]*std_landmark[0]) + (y_diff*y_diff)/(2 * std_landmark[1]*std_landmark[1]);
				// calculate weight using normalization terms and exponent
				particles[i].weight += gauss_norm * exp(-exponent);
			}  
		}
		if (particles[i].weight > max_weight)
		{
			max_weight = particles[i].weight;
		}
	}
	// Normalise all weights from 0 to 1
	for (int i = 0; i<num_particles; i++)
	{
		particles[i].weight = particles[i].weight/max_weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	std::vector<Particle> new_particles;
	uniform_real_distribution<double> r(0,1);
	discrete_distribution<int> start(0, num_particles);
	int index = start(gen);
	for (int i = 0; i < num_particles; i++)
	{
		double test = particles[index].weight;
		double step = r(gen) * 2;
		while (test<step)
		{
			index++;
			index = index % num_particles;
			test+=particles[index].weight;
		}
		new_particles.push_back(particles[index]);
	}
	particles = new_particles;
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

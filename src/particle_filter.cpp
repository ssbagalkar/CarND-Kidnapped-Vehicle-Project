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
#include <map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	//create a normal distribution for x,y position and theta,the heading angle
	normal_distribution <double> dist_x (x, std[0]);
	normal_distribution <double> dist_y (y, std[1]);
	normal_distribution <double> dist_theta (theta, std[2]);

	//decide on number of particles
	num_particles = 5; //start with 100.Later maybe adjusted
	
	
	//set length of weights to num_particles
	weights.resize(num_particles);

	//reserve length of particles to num_particles
	particles.resize(num_particles);

	// Use a psedo random number generator
	default_random_engine gen;

	//Generate num_particles particles,add noise and weights
	for (int ii = 0; ii < num_particles; ++ii)
	{
		particles.at(ii).x = dist_x(gen);
		particles.at(ii).y = dist_y(gen);
		particles.at(ii).theta = dist_theta(gen);
		particles.at(ii).weight = 1.0;
	}

	// set initialized flag to true
	is_initialized = true;

	//Print out the initialized particles
	cout << "Initial Particle : " << "x: " << x << endl;
	cout << "y: " << y << endl;
	cout << "theta: " << theta << endl;

	// Print initialization
	cout << "Initialization successful" << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Use a psedo random number generator
	default_random_engine gen;

	// depending if yaw rate is zero,use different set of equations to predict the x,y positions
	// and heading angle which is yaw angle theta
	for (int ii = 0; ii < num_particles; ++ii) {
		if (fabs(yaw_rate) < 0.0000001)
		{
			// x_pred = x_prev + velocity * delta_t * cos (theta)
			double x_pred = particles.at(ii).x + (velocity * delta_t * cos(particles.at(ii).theta));

			// add random gaussian noise for x position
			normal_distribution <double> dist_x_pred(0, std_pos[0]);

			// assign the predicted measurement to x position with added random gaussian noise
			particles.at(ii).x = x_pred + dist_x_pred(gen);

			// y_pred = x_prev + velocity * delta_t * sin (theta)
			double y_pred = particles.at(ii).y + (velocity * delta_t * sin(particles.at(ii).theta));

			// add random gaussian noise for x position
			normal_distribution <double> dist_y_pred(0, std_pos[1]);

			// assign the predicted measurement to x position with added random gaussian noise
			particles.at(ii).y = y_pred +  dist_y_pred(gen);
			
			//theta_pred = theta_prev.So,no need to change the equation
		}
		// if yaw rate is not equal to 0
		else
		{
			// x_pred = x_prev + velocity * delta_t * cos (theta)
			double x_pred = particles.at(ii).x + (velocity/yaw_rate)*(sin((particles.at(ii).theta) + (yaw_rate * delta_t)) - (sin(particles.at(ii).theta)));

			// add random gaussian noise for x position
			normal_distribution <double> dist_x_pred(0, std_pos[0]);

			//cout << "normal random noise" << dist_x_pred(gen) << endl;
			// assign the predicted measurement to x position with added random gaussian noise
			particles.at(ii).x = x_pred + dist_x_pred(gen);
			// y_pred = x_prev + velocity * delta_t * sin (theta)
			double y_pred = particles.at(ii).y + (velocity / yaw_rate)*((cos(particles.at(ii).theta)) - (cos(particles.at(ii).theta + (yaw_rate * delta_t))));

			// add random gaussian noise for x position
			normal_distribution <double> dist_y_pred(0, std_pos[1]);

			
			// assign the predicted measurement to x position with added random gaussian noise
			particles.at(ii).y = y_pred +  dist_y_pred(gen);
			
			// theta_pred = theta_prev + yaw_rate * delta_t
			double theta_pred = particles.at(ii).theta + ((yaw_rate * delta_t));

			// add random gaussian noise for theta 
			normal_distribution <double> dist_theta_pred(0, std_pos[2]);

			// assign the predicted measurement to x position with added random gaussian noise
			particles.at(ii).theta = theta_pred + dist_theta_pred(gen);
			

		}
	}
	
	
}

//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
//	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
//	//   observed measurement to this particular landmark.
//	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
//	//   implement this method and use it as a helper during the updateWeights phase.
//
//}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs>& predicted, std::vector<LandmarkObs>& observations, Map map_landmarks)
{

	for (int sensorCount = 0; sensorCount < observations.size(); ++sensorCount)
	{
		double min_dist = INFINITY;
		// transform each sensor measurement ,which is in local(car) coordinates to global(map) coordinates
		/*
		Equation to transform car coordinates to map coordinates
		|Xmap| = |cos(theta)  -sin(theta)   Xparticle |      | Xcar |
		|Ymap| = |sin(theta)   cos(theta)   Yparticle |   X  | Ycar |
		|  1 | = |    0             0            1    |      |   1  |

		*/
		//// use above equation for each sensor
		//observations[sensorCount].x = particles.at(ii).x + (cos(particles.at(ii).theta) * (observations[sensorCount].x)) - (sin(particles.at(ii).theta) * (observations[sensorCount].y));
		//observations[sensorCount].y = particles.at(ii).y + (sin(particles.at(ii).theta) * (observations[sensorCount].x)) + (cos(particles.at(ii).theta) * (observations[sensorCount].y));

		for (int landmarkCount = 0; landmarkCount < map_landmarks.landmark_list.size(); ++landmarkCount)
		{
			// TODO## restrict far away landmarks suing sensor range

			// Calculate euclidean distance between each landmark pos and sensor pos
			//double dist_euclid = sqrt(pow(map_landmarks.landmark_list[landmarkCount].x_f - observations[sensorCount].x, 2) + 
			//					 pow(map_landmarks.landmark_list[landmarkCount].y_f - observations[sensorCount].y, 2));
			double dist_euclid = dist(map_landmarks.landmark_list[landmarkCount].x_f, map_landmarks.landmark_list[landmarkCount].y_f,
				observations[sensorCount].x, observations[sensorCount].y);

			// if this distance is less than min distance,associate current particle with that landmark index
			if (dist_euclid < min_dist)
			{
				// change the min distance to calculated euclidean distance  
				min_dist = dist_euclid;

				// add the landmark index to predicted vector id
				observations[sensorCount].id = map_landmarks.landmark_list[landmarkCount].id_i;
			}

		}

		// now add the sensor measurement with its associated id to predicted vector
		predicted.push_back(observations[sensorCount]);
	}
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	// loop through each particle to calculate its weight 
	for (int ii = 0; ii < num_particles; ++ii)
	{
		double weight_for_each_particle = 1.0;

		// create a empty vector field predicted
		predicted.clear(); // probably have to initialize it before.Let's see

		// set size of transformed observations vector
		transformed_obs.resize(observations.size());
		cout << "Transformations" << endl;
		// for each observation from sensor,convert it to map coordinates
		for (int sensorCount = 0; sensorCount < observations.size(); ++sensorCount)
		{
			// transform each sensor measurement ,which is in local(car) coordinates to global(map) coordinates
			/*
			//	Equation to transform car coordinates to map coordinates
			//	|Xmap| = |cos(theta)  -sin(theta)   Xparticle |      | Xcar |
			//	|Ymap| = |sin(theta)   cos(theta)   Yparticle |   X  | Ycar |
			//	|  1 | = |    0             0            1    |      |   1  |
			//	
			//	*/
			// use above equation for each sensor to transform observations
			transformed_obs[sensorCount].x = particles.at(ii).x + (cos(particles.at(ii).theta) * (observations[sensorCount].x)) - (sin(particles.at(ii).theta) * (observations[sensorCount].y));
			transformed_obs[sensorCount].y = particles.at(ii).y + (sin(particles.at(ii).theta) * (observations[sensorCount].x)) + (cos(particles.at(ii).theta) * (observations[sensorCount].y));

			cout << "observation: (" << observations[sensorCount].x <<" " << observations[sensorCount].y << ") --->" <<
				"transfomed_obs: (" << transformed_obs[sensorCount].x << " " << transformed_obs[sensorCount].y << ")" << endl;
		}
		
		// apply data association for each sensor measurement and create a predicted vector for each sensor measurement
		dataAssociation(predicted, transformed_obs, map_landmarks);

		// calculate weight using multivariate gaussian probability distribution
		for (int ii = 0; ii < observations.size(); ++ii)
		{
			/* start process of assigning weights to each particle 
			using multivariate gaussian probability distribution*/

			// retrieve landmark x and y position associated with ith sensor 
			// measurement based on the index that is stored in predicted vector
			int associated_index = predicted[ii].id;

			// retrieve the x and y positions of the landmark
			double x_landmark = map_landmarks.landmark_list[associated_index-1].x_f;
			double y_landmark = map_landmarks.landmark_list[associated_index-1].y_f;

			
			// calculate normalizer
			double gauss_norm = (1 / (2 * M_PI * std_landmark[0] * std_landmark[1]));

			//calculate the exponent
			double exponent = ((pow(predicted[ii].x - x_landmark,2))/(2 * pow(std_landmark[0],2))) + 
				((pow(predicted[ii].y - y_landmark, 2)) / (2 * pow(std_landmark[1], 2)));

			 weight_for_each_particle = weight_for_each_particle * gauss_norm * exp(-exponent);

		}

		particles.at(ii).weight = weight_for_each_particle;
		weights.at(ii) = weight_for_each_particle;
	} 

	

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::random_device rd;
	std::mt19937 gen(rd());
	std::discrete_distribution<> d(weights.begin(), weights.end());
	std::map<int, int> m;
	for (int n = 0; n<num_particles; ++n) {
		resampled_particles.push_back(particles[d(gen)]);
	}

	// clear all particles and replace with resampled ones
	//particles.erase(particles.begin(),particles.end());
	for (int ii = 0; ii < num_particles; ++ii)
	{
		particles.at(ii).x = resampled_particles.at(ii).x;
		particles.at(ii).y = resampled_particles.at(ii).y;
		particles.at(ii).theta = resampled_particles.at(ii).theta;
		particles.at(ii).weight = resampled_particles.at(ii).weight;
	}

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle/particle_filter.h"
using namespace std;

static  default_random_engine gen;

/*
* TODO
* This function initialize randomly the particles
* Input:
*  std - noise that might be added to the position
*  nParticles - number of particles
*/
void ParticleFilter::init_random(double std[],int nParticles) {

}

// This function initialize the particles using an initial guess
void ParticleFilter::init(double x, double y, double theta, double std[],int nParticles) {
    num_particles = nParticles;
    normal_distribution<double> dist_x(-std[0], std[0]); //random value between [-noise.x,+noise.x]
    normal_distribution<double> dist_y(-std[1], std[1]);
    normal_distribution<double> dist_theta(-std[2], std[2]);

    for(int i=0;i<num_particles;i++){
        Particle p;
        p.id = i;
        p.x = x+dist_x(gen);
        p.y = y+dist_y(gen);
        p.theta = theta+dist_theta(gen);
        p.weight = 1; 
        particles.push_back(p);
    }
    
    is_initialized=true;
}

/*
* TODO
* The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep
* Input:
*  delta_t  - time elapsed beetween measurements
*  std_pos  - noise that might be added to the position
*  velocity - velocity of the vehicle
*  yaw_rate - current orientation
* Output:
*  Updated x,y,theta position
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    //for each particle
    for (int i=0; i<particles.size(); i++)
    {
        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;

        if (fabs(yaw_rate) < 0.00001) 
        {
            x += velocity * delta_t * cos(theta);
            y += velocity * delta_t * sin(theta);
                        //TODO
        }
        else
        { 
            x += velocity / yaw_rate *(sin(theta + yaw_rate * delta_t) - sin(theta));
            y += velocity / yaw_rate *(cos(theta) - cos(theta + yaw_rate * delta_t));
            theta += yaw_rate * delta_t;
            //TODO

        }   
        normal_distribution<double> dist_x(0, std_pos[0]); //the random noise cannot be negative in this case
        normal_distribution<double> dist_y(0, std_pos[1]);
        normal_distribution<double> dist_theta(0, std_pos[2]);
        //add the noise to the particles position (x,y,theta)

        particles[i].x = x + dist_x(gen);
        particles[i].y = y+ dist_y(gen);
        particles[i].theta = theta + dist_theta(gen);

	//
}
}
/*
* TODO
* This function associates the landmarks from the MAP to the landmarks from the OBSERVATIONS
* Input:
*  mapLandmark   - landmarks of the map
*  observations  - observations of the car
* Output:
*  Associated observations to mapLandmarks (perform the association using the ids)
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> mapLandmark, std::vector<LandmarkObs>& observations)
{
   //TODO
   //TIP: Assign to observations[i].id the id of the landmark with the smallest euclidean distance
    /*
    For each observation in J //use the size() function
        Min_dist=INFINITY
        For each landmark in K
            compute the difference in x between the landmark and measurement
            compute the difference in y between the landmark and measurement
            compute the euclidean distance 
            if(euclidean distance <min_dist)
                save the minimum distance
                save the index of the landmark that has the minimum distance 
            end if
        End for
        Observations[J].id=index
    End for
    */
    for (int i=0; i < observations.size(); i++)
    {
        double min_dist =numeric_limits<double>::max();
        int map_id = -1;
        for(int j=0; j < mapLandmark.size(); j++)
        {
            double x_diff = observations[i].x - mapLandmark[j].x;
            double y_diff = observations[i].y - mapLandmark[j].y;
            double dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

            if (dist < min_dist)
            {
                min_dist = dist;
                map_id = mapLandmark[j].id;
            }
        }
        observations[i].id = map_id;
    }
}

/*
* TODO
* This function transform a local (car) observation into a global (map) coordinates
* Input:
*  observation   - A single landmark observation
*  p             - A single particle
* Output:
*  local         - transformation of the observation from local coordinates to global
*/
LandmarkObs transformation(LandmarkObs observation, Particle p){
    LandmarkObs global;
    
    global.id = observation.id; 
    global.x = p.x + observation.x * cos(p.theta) - observation.y * sin(p.theta);
    global.y = p.y + observation.x * sin(p.theta) + observation.y * cos(p.theta);
    
    return global;
}

/*
* TODO
* This function updates the weights of each particle
* Input:
*  std_landmark   - Sensor noise
*  observations   - Sensor measurements
*  map_landmarks  - Map with the landmarks
* Output:
*  Updated particle's weight (particles[i].weight *= w)
*/
void ParticleFilter::updateWeights(double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

    //Creates a vector that stores tha map (this part can be improved)
    std::vector<LandmarkObs> mapLandmark;

    for(int j=0;j<map_landmarks.landmark_list.size();j++){
        mapLandmark.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f});
    }
    for(int i=0;i<particles.size();i++){

        // Before applying the association we have to transform the observations in the global coordinates
        std::vector<LandmarkObs> transformed_observations;
        //TODO: for each observation transform it
        for (int k=0; k < observations.size(); k++)
        {
            transformed_observations.push_back(transformation(observations[k], particles[i]));

        }
        //TODO: perform the data association (associate the landmarks to the observations)
        dataAssociation(mapLandmark, transformed_observations);

        particles[i].weight = 1.0;
        // Compute the probability
		//The particles final weight can be represented as the product of each measurement’s Multivariate-Gaussian probability density
		//We compute basically the distance between the observed landmarks and the landmarks in range from the position of the particle
        for(int k=0;k<transformed_observations.size();k++){
            double obs_x,obs_y,l_x,l_y;
            obs_x = transformed_observations[k].x;
            obs_y = transformed_observations[k].y;
            //get the associated landmark 
            for (int p = 0; p < mapLandmark.size(); p++) {
                if (transformed_observations[k].id == mapLandmark[p].id) {
                    l_x = mapLandmark[p].x;
                    l_y = mapLandmark[p].y;
                }
            }	
			// How likely a set of landmarks measurements are, given a prediction state of the car 
            double w = exp( -( pow(l_x-obs_x,2)/(2*pow(std_landmark[0],2)) + pow(l_y-obs_y,2)/(2*pow(std_landmark[1],2)) ) ) / ( 2*M_PI*std_landmark[0]*std_landmark[1] );
            particles[i].weight *= w;
        }

    }    
}

/*
* TODO
* This function resamples the set of particles by repopulating the particles using the weight as metric
*/
void ParticleFilter::resample() {
    
    uniform_int_distribution<int> dist_distribution(0,num_particles-1);
    double beta  = 0.0;
    vector<double> weights;
    int index = dist_distribution(gen);
    vector<Particle> new_particles;

    for(int i=0;i<num_particles;i++)
        weights.push_back(particles[i].weight);
																
    float max_w = *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> uni_dist(0.0, max_w);

    //TODO write here the resampling technique (feel free to use the above variables)
    for (int i=0; i < num_particles; i++)
    {
        beta += uni_dist(gen) * 2.0;
        while (weights[index] < beta)
        {
            beta-= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);

    }
    particles = new_particles;
}



#include "Robot.hpp"
#include <cmath>
#include <iostream>
using namespace std;

Robot::Robot(
				double x,
				double y,
				double w,
				double h,
				double r,
				double v,
				double a,
				vector<pair<double, double>> shape,
				vector<Robot> *flock,
				double radius_rep,
				double radius_ori,
				double radius_att
			):Wired(x,y,w,h,r,shape){
	this->v				= v;
	this->a				= a;
	this->gx			= 0;
	this->gy			= 0;
	this->radius_rep	= radius_rep;
	this->radius_ori	= radius_ori;
	this->radius_att	= radius_att;
	this->flock			= flock;
	this->selected = false;
}

Robot::~Robot(){};

void Robot::respawn(double x,double y){
	this->x = x;
	this->y = y;
}

void Robot::set_goal_target_pos(double gx,double gy){
	this->gx = gx;
	this->gy = gy;
}

void Robot::update(){

	// Update heading and velocities
	double delta = rad_to_deg(swarm())-this->t;
	angle_wrap(delta);

	this->t += this->a*delta;
	angle_wrap(this->t);

	// Send velocity commands
	this->x += v*cos(deg_to_rad(this->t));
	this->y += v*sin(deg_to_rad(this->t));

};

// Get agents between radii
set<Robot*> Robot::get_neighbors(double radiusMax, double radiusMin = 0.0){
	set<Robot*> nbors;
	for(auto &r : *flock)
		if(this != &r && distance_to_robot(&r) <= radiusMax && distance_to_robot(&r) >= radiusMin)
			nbors.insert(&r);
	return nbors;
}


// Update flocking neighbors
 void Robot::update_neighbors(){
	this->neighbor_rep = this->get_neighbors(this->radius_rep);
	this->neighbor_ori = this->get_neighbors(this->radius_ori);
	this->neighbor_att = this->get_neighbors(this->radius_att, this->radius_rep);
}

// Returns next heading (in radians) based on local interactions
double Robot::swarm(){
	double w_rep = this->wall_repulsion(WORLD_SIZE_X, WORLD_SIZE_Y);
	if(w_rep == w_rep)
		return w_rep;
	else
		return this->reynolds_rules();
}

pair<double, double> Robot::compute_force(set<Robot*> &neighbors){
	pair<double,double> force(0.0,0.0);
	for(auto &r : neighbors){
		double dx = r->x - this->x;
		double dy = r->y - this->y;
		double d2 = pow(dx,2) + pow(dy,2);
		force.first  += dx/d2;
		force.second += dy/d2;
	}
	return force;
}

// Returns desired heading based on swarming rules
double Robot::reynolds_rules(){

	// Update neighbor sets for radius_rep, radius_ori, radius_att
	update_neighbors();

	// Repulsion Vector
	pair<double,double> rep = compute_force(neighbor_rep);

	// Attraction Vector
	pair<double,double> att = compute_force(neighbor_att);

	// Orientation Vector
	pair<double,double> ori(0.0,0.0);
	for(auto &r : neighbor_ori){
		double d = distance_to_robot(r);
		double th = deg_to_rad(r->t);
		ori.first  += cos(th)/d;
		ori.second += sin(th)/d;
	}

	// Add up all velocities, normalized and weighted by distance
	double x = -rep.first  +att.first  +ori.first;
	double y = -rep.second +att.second +ori.second;

	// Go straight in the absence of neighbors
	if(x == 0 && y == 0)
		return deg_to_rad(this->t);
	else
		return atan2(y, x);
}

// Returns heading based on wall repulsions
double Robot::wall_repulsion(double xlim, double ylim){
	if(abs(x) >= xlim) return atan2(0.0, xlim-x);
	if(abs(y) >= ylim) return atan2(ylim-y, 0.0);
	return NAN;
}

double Robot::distance_to_point(double x, double y){
	return sqrt(pow(this->x-x, 2) + pow(this->y-y, 2));
}

double Robot::distance_to_robot(Robot *robot){
	return this->distance_to_point(robot->x, robot->y);
}

 bool Robot::check_col(){
	for(auto const &r : *flock)
		if(this != &r){
			if(this->x+this->w/2 < r.x-r.w/2) continue;
			if(this->x-this->w/2 > r.x+r.w/2) continue;
			if(this->y+this->h/2 < r.y-r.h/2) continue;
			if(this->y-this->h/2 > r.y+r.h/2) continue;
			return true;
		}	
	return false;
}

void Robot::render_robot(){
	render(1,selected);
};

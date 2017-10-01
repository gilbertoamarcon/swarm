#include "Robot.hpp"
#include <cmath>

#include <iostream>
using namespace std;

Robot::Robot(){
	this->vel = 0;
	this->steer = 0;
	this->lx = 0;
	this->ly = 0;
	this->lt = 0;
	this->ox = 0;
	this->oy = 0;
	this->ot = 0;
	this->vx = 0;
	this->vy = 0;
	this->vt = 0;
	this->neighbor_rep;
	this->neighbor_ori;
	this->neighbor_att;
	this->selected = false;
}

Robot::~Robot(){};

int Robot::init(
					double x,
					double y,
					double w,
					double h,
					double r,
					double vel,
					double s,
					vector<pair<double, double>> shape,
					vector <Robot> *flock,
					double radius_rep,
					double radius_ori,
					double radius_att
				){
	this->x		= x;
	this->y		= y;
	this->w		= w;
	this->h		= h;
	this->t		= t;
	this->vel	= vel;
	this->steer	= s;
	this->shape	= shape;
	this->flock	= flock;
	this->radius_rep	= radius_rep;
	this->radius_ori	= radius_ori;
	this->radius_att	= radius_att;
}

void Robot::respawn(double x,double y){
	this->x = x;
	this->y = y;
}

void Robot::set_goal_target_pos(double gx,double gy){
	this->lx = gx;
	this->ly = gy;
	this->lt = 0;
}

void Robot::update(){

	// Get desired heading based on swarming interactions
	double td = swarm();

	// Update heading and velocities
	double delta = rad_to_deg(td)-this->t;
	angle_wrap(delta);

	this->t += this->steer*delta;
	angle_wrap(this->t);

	// Update positions
	vx = vel*cos(deg_to_rad(this->t));
	vy = vel*sin(deg_to_rad(this->t));

	// Send velocity commands
	this->x += vx;
	this->y += vy;

};

// Get agents between radii
vector<Robot*> Robot::get_neighbors(double radiusMax, double radiusMin = 0.0){
	vector<Robot*> nbors;
	for(auto &r : *flock)
		if(this != &r && distance_to_robot(&r) <= radiusMax && distance_to_robot(&r) >= radiusMin)
			nbors.push_back(&r);
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

void Robot::compute_force(vector<Robot*> &neighbors, double &f_x, double &f_y){
	f_x = 0.0;
	f_y = 0.0;
	for(auto &r : neighbors){
		double dx = r->x - this->x;
		double dy = r->y - this->y;
		double d2 = pow(dx,2) + pow(dy,2);
		f_x += dx/d2;
		f_y += dy/d2;
	}
}

// Returns desired heading based on swarming rules
double Robot::reynolds_rules(){

	// Update neighbor sets for radius_rep, radius_ori, radius_att
	update_neighbors();

	// Repulsion Vector
	double rep_x = 0;
	double rep_y = 0;
	compute_force(neighbor_rep,rep_x,rep_y);

	// Attraction Vector
	double att_x = 0;
	double att_y = 0;
	compute_force(neighbor_att,att_x,att_y);

	// Orientation Vector
	double ori_x = 0;
	double ori_y = 0;
	for(auto &r : neighbor_ori){
		double d = distance_to_robot(r);
		double th = deg_to_rad(r->t);
		ori_x += cos(th)/d;
		ori_y += sin(th)/d;
	}

	// Add up all velocities, normalized and weighted by distance
	double x = - rep_x + att_x + ori_x;
	double y = - rep_y + att_y + ori_y;

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

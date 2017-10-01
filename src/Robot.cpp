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

	// Update neighbor sets for radius_rep, radius_ori, radius_att
	update_neighbors();

	// Get desired heading based on swarming interactions
	double td = swarm();

	// Update heading and velocities
	// Wire representation requires conversion to degrees
	// Coordinate frame corrections -> Not sure if a simpler way exists...

	// Angle warp around
	if(abs(this->t) > 180)
		this->t -= 360*(2*(this->t>0)-1);

	double delta = rad_to_deg(td)-this->t;
	if(delta <= 180 && delta >= -180)
		this->t += this->steer*(delta);
	else{
		if(delta < -180)
			this->t += this->steer*(delta+360);
		if(delta > 180)
			this->t += this->steer*(delta-360);
	}

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
	double wRep = this->wall_repulsion(WORLD_SIZE_X, WORLD_SIZE_Y);
	if(wRep == wRep)
		return wRep;
	else
		return this->reynolds_rules();
}

// Returns desired heading based on swarming rules
double Robot::reynolds_rules(){

	// Repulsion Vector
	double repX = 0;
	double repY = 0;
	for(auto const &r : neighbor_rep){
		double dx =  this->x - r->x;
		double dy =  this->y - r->y;
		double d = sqrt(pow(dx,2) + pow(dy,2));
		repX += dx/d/d;	
		repY += dy/d/d;
	}

	// Orientation Vector
	double oriX = 0;
	double oriY = 0;
	double sum = 0;
	int cnt = 0;
	for(auto const &r : neighbor_ori){
		double dx =  this->x - r->x;
		double dy =  this->y - r->y;
		double d = sqrt(pow(dx,2) + pow(dy,2));
		double th = deg_to_rad(r->t);
		oriX += cos(th)/d;
		oriY += sin(th)/d;
	}

	// Attraction Vector
	double atrX = 0;
	double atrY = 0;
	for(auto const &r : neighbor_att){
		double dx = r->x - this->x;
		double dy = r->y - this->y;
		double d = sqrt(pow(dx,2) + pow(dy,2));
		atrX += dx/d/d;	
		atrY += dy/d/d;
	}

	// Add up all velocities, normalized and weighted by distance
	double x = repX + atrX + oriX;
	double y = repY + atrY + oriY;

	// Go straight in the absence of neighbors
	if(x == 0 && y == 0)
		return deg_to_rad(this->t);
	else
		return atan2(y, x);
}

// Returns heading based on wall repulsions
double Robot::wall_repulsion(double xlim, double ylim){
	double wallX = 0;
	double wallY = 0;
	bool noWalls = true;
	if(this->x >= xlim){noWalls = false; wallX = xlim - this->x;}
	if(this->x <= -xlim){noWalls = false; wallX = xlim - this->x;}
	if(this->y >= ylim){noWalls = false; wallY = ylim - this->y;}
	if(this->y <= -ylim){noWalls = false; wallY = ylim - this->y;}
	if(noWalls) {return NAN;}
	return atan2(wallY, wallX);
}

double Robot::distance_to_point(double x, double y){
	return sqrt(pow(this->x - x, 2) + pow(this->y - y, 2));
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

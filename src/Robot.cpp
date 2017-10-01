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
	this->neighbor_rep = {};
	this->neighbor_ori = {};
	this->neighbor_att = {};
	this->selected = false;
}

Robot::Robot(int id){
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
	this->neighbor_rep = {};
	this->neighbor_ori = {};
	this->neighbor_att = {};
	this->selected = false;
	this->id = id;
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

void Robot::setGoalTargetPos(double gx,double gy){
	this->lx = gx;
	this->ly = gy;
	this->lt = 0;
}

void Robot::update(){

	// Update neighbor sets for radius_rep, radius_ori, radius_att
	updateNeighbors();

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
 vector<int> Robot::getNeighbors(double radiusMax, double radiusMin = 0.0){
	vector <int> nbors = {};
	for(auto const &r : *flock)
		if(distanceToRobot(r) < radiusMax && distanceToRobot(r) > radiusMin)
			nbors.push_back(r.id);
	return nbors;
}


// Update flocking neighbors
 void Robot::updateNeighbors(){
	this->neighbor_rep = this->getNeighbors(this->radius_rep);
	this->neighbor_ori = this->getNeighbors(this->radius_ori);
	this->neighbor_att = this->getNeighbors(this->radius_att, this->radius_rep);
}

// Returns next heading (in radians) based on local interactions
double Robot::swarm(){
	double wRep = this->wallRepulsion(WORLD_SIZE_X, WORLD_SIZE_Y);
	if(wRep == wRep)
		return wRep;
	else
		return this->reynoldsRules();
}

// Returns desired heading based on swarming rules
double Robot::reynoldsRules(){
	// Repulsion Vector
	double repX = 0;
	double repY = 0;
	for(int i = 0; i < this->neighbor_rep.size(); i++){
		double dx =  this->x - this->flock->at(neighbor_rep[i]).x;
		double dy =  this->y - this->flock->at(neighbor_rep[i]).y;
		double d = sqrt(pow(dx,2) + pow(dy,2));
		repX += dx/d/d;	
		repY += dy/d/d;
	}
	// Orientation Vector
	double oriX = 0;
	double oriY = 0;
	double sum = 0;
	int cnt = 0;
	for(int i = 0; i < this->neighbor_ori.size(); i++){
		double dx =  this->x - this->flock->at(neighbor_ori[i]).x;
		double dy =  this->y - this->flock->at(neighbor_ori[i]).y;
		double d = sqrt(pow(dx,2) + pow(dy,2));
		double th = deg_to_rad(this->flock->at(neighbor_ori[i]).t);
		oriX += cos(th)/d;
		oriY += sin(th)/d;
	}
	// Attraction Vector
	double atrX = 0;
	double atrY = 0;
	for(int i = 0; i < this->neighbor_att.size(); i++){
		double dx = this->flock->at(neighbor_att[i]).x - this->x;
		double dy = this->flock->at(neighbor_att[i]).y - this->y;
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
double Robot::wallRepulsion(double xlim, double ylim){
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

double Robot::distanceToPoint(double x, double y){
	return sqrt(pow(this->x - x, 2) + pow(this->y - y, 2));
}

double Robot::distanceToRobot(vector<Robot> *flock, int id){
	return this->distanceToPoint(flock->at(id).x, flock->at(id).y);
}

double Robot::distanceToRobot(Robot robot){
	return this->distanceToPoint(robot.x, robot.y);
}

 bool Robot::checkCol(){
	for(auto r : *flock)
		if(this->id != r.id){
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

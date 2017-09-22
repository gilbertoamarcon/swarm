#include "Robot.hpp"
#include <cmath>

Robot::Robot(){
	this->vel = 0;
	this->lx = 0;
	this->ly = 0;
	this->lt = 0;
	this->ox = 0;
	this->oy = 0;
	this->ot = 0;
	this->vx = 0;
	this->vy = 0;
	this->vt = 0;
	this->selected = false;
};

Robot::Robot(int id){
	this->vel = 0;
	this->lx = 0;
	this->ly = 0;
	this->lt = 0;
	this->ox = 0;
	this->oy = 0;
	this->ot = 0;
	this->vx = 0;
	this->vy = 0;
	this->vt = 0;
	this->selected = false;
	this->id = id;
};

Robot::~Robot(){};

int Robot::init(double x,double y,double w,double h,double t,double vel,vector<pair<double, double>> shape){
	wire.x = x;
	wire.y = y;
	wire.w = w;
	wire.h = h;
	wire.t = t;
	this->vel = vel;
	return wire.init(x,y,w,h,t,shape);
};

bool Robot::respawn(double x,double y){
	wire.x = x;
	wire.y = y;
};

void Robot::setGoalTargetPos(double gx,double gy){
	this->lx = gx;
	this->ly = gy;
	this->lt = 0;
}

void Robot::update(bool col){

	// Distance to target
	double dlx = lx - wire.x;
	double dly = ly - wire.y;
	double dl = sqrt(pow(dlx,2)+pow(dly,2));

	// Direction to target
	double rx = dlx/dl;
	double ry = dly/dl;

	// Angle to target
	if(dlx == 0.00)
		wire.t = 0.00;
	else{
		wire.t = (180/PI)*atan(dly/dlx);
		if(dlx < 0)
			wire.t += 180;
	}

	// Stop if target was reached
	if(dl < DEFAULT_TOL*wire.w){
		rx = 0;
		ry = 0;
	}

	// Col check
	if(col){
		wire.x = ox;
		wire.y = oy;
	}else{
		ox = wire.x;
		oy = wire.y;
	}

	vx = vel*cos((PI/180)*wire.t);
	vy = vel*sin((PI/180)*wire.t);

	wire.x += vx;
	wire.y += vy;

};

int Robot::getNeighbors(){
	vector <int> nbors = {};
	for (int i = 0; i < this->flock.size(); i ++){
		Robot current = this->flock[i];
		printf("ID: %d ", current.id);
	}
	return 0;
 }

 double Robot::distanceToPoint(double x, double y){
 	return sqrt(pow(this->ox - x, 2) + pow(this->oy - y, 2));
 }

 double Robot::distanceToRobot(vector<Robot> flock, int id){
 	return this->distanceToPoint(flock[id].ox, flock[id].oy);
 }

void Robot::render(){
	wire.render(1,selected);
};

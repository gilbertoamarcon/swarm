#include "Robot.hpp"
#include <cmath>

#include <iostream>
using namespace std;

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
	this->nRep = {};
	this->nOri = {};
	this->nAtr = {};
	this->selected = false;
}

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
	this->nRep = {};
	this->nOri = {};
	this->nAtr = {};
	this->selected = false;
	this->id = id;
}

Robot::~Robot(){};

int Robot::init(double x,double y,double w,double h,double t,double vel,vector<pair<double, double>> shape){
	wire.x = x;
	wire.y = y;
	wire.w = w;
	wire.h = h;
	wire.t = t;
	this->vel = vel;
	return wire.init(x,y,w,h,t,shape);
}

void Robot::respawn(double x,double y){
	wire.x = x;
	wire.y = y;
}

void Robot::setGoalTargetPos(double gx,double gy){
	this->lx = gx;
	this->ly = gy;
	this->lt = 0;
}

void Robot::update(bool col){

	// Distance to target
	// double dlx = lx - wire.x;
	// double dly = ly - wire.y;
	// double dl = sqrt(pow(dlx,2)+pow(dly,2));

	// Direction to target
	// double rx = dlx/dl;
	// double ry = dly/dl;

	// Angle to target
	// if(dlx == 0.00)
	// 	wire.t = 0.00;
	// else{
	// 	wire.t = (180/PI)*atan(dly/dlx);
	// 	if(dlx < 0)
	// 		wire.t += 180;
	// }

	// Stop if target was reached
	// if(dl < DEFAULT_TOL*wire.w){
	// 	rx = 0;
	// 	ry = 0;
	// }

	// Col check
	// if(col){
	// 	wire.x = ox;
	// 	wire.y = oy;
	// }else{
	// 	ox = wire.x;
	// 	oy = wire.y;
	// }


	// Update neighbor sets for rRep, rOri, rAtr
	updateNeighbors();
	// Get desired heading based on swarming interactions
	double td = swarm();
	// Update heading and velocities
	// Wire representation requires conversion to degrees
	// 0.05 is the heading change rate --> TODO: Make this a parameter

	if (wire.t > 180)
		wire.t -= 360;

	if (wire.t < -180)
		wire.t += 360;

	double step = 0.05;
	if (td*180/PI-wire.t <= 180 && td*180/PI-wire.t >= -180)
		wire.t += step*(td*180/PI-wire.t);
	else{
		if (td*180/PI-wire.t < -180)
			wire.t += step*(td*180/PI-wire.t+360);
		if (td*180/PI-wire.t > 180)
			wire.t += step*(td*180/PI-wire.t-360);
	}

	// cout << "Desired: " << td*180/PI << "True: " << wire.t << endl;

	vx = vel*cos(wire.t*PI/180);
	vy = vel*sin(wire.t*PI/180);
	// Send velocity commands
	// cout << "ID: " << id << " " << wire.x << " " << wire.y << endl;
	wire.x += vx;
	wire.y += vy;


};

vector<int> Robot::getNeighbors(double radius){
	// cout << "Getting Nbors in radius: " << radius << endl;
	// cout << "Flock size: " << this->flock.size() << endl;
	// cout << "My ID: " << this->id << endl;
	vector <int> nbors = {};
	for (int i = 0; i < this->flock->size(); i++){
		Robot current = this->flock->at(i);
		if (current.id != this->id && distanceToRobot(current)<=radius){
			nbors.push_back(current.id);
			// cout << "Nbor ID: " << current.id << endl;
		}
	}
	return nbors;
 }

 void Robot::updateNeighbors(){
 	// TODO: Members of nRep could not exist in nOri or nAtr
 	this->nRep = this->getNeighbors(this->rRep);
 	this->nOri = this->getNeighbors(this->rOri);
 	this->nAtr = this->getNeighbors(this->rAtr);
 	// cout << this->nRep.size() << this->nOri.size() << this->nAtr.size() << endl;
 }

 double Robot::swarm(){
 	// Returns next heading (in radians) based on local interactions
 	// cout << "Rules: " << this->reynoldsRules() << "Wrep: " << this->wallRepulsion(250, 250) << endl;

 	double wRep = this->wallRepulsion(250, 250);

 	if (wRep == wRep)
 		return wRep;
 	else
 		return this->reynoldsRules();

 }

 double Robot::reynoldsRules(){
 	double repX = 0;
 	double repY = 0;
 	for (int i = 0; i < this->nRep.size(); i++){
 		double dx =  this->wire.x - this->flock->at(nRep[i]).wire.x;
 		double dy =  this->wire.y - this->flock->at(nRep[i]).wire.y;
 		double d = sqrt(pow(dx,2) + pow(dy,2));
		repX += dx/d/d;	
		repY += dy/d/d;	
 	}

 	// double oriH = 0;
 	// double sum = 0;
 	// double sumd = 0;
 	// double prod = 1;
 	// for (int i = 0; i < this->nOri.size(); i++){
 	// 	double dx =  this->wire.x - this->flock[nOri[i]].wire.x;
 	// 	double dy =  this->wire.y - this->flock[nOri[i]].wire.y;
 	// 	double d = sqrt(pow(dx,2) + pow(dy,2));
 	// 	double th = this->flock[nOri[i]].wire.t;
		// sum += th/d;
		// sumd += d;	
		// prod *= d;
 	// }

  	double oriH = 0;
 	double sum = 0;
 	int cnt = 0;
 	for (int i = 0; i < this->nOri.size(); i++){
 		// cout << " i: " << i << "Robot: " << nOri[i] << endl;
 		// cout << "XY " << this->flock[nOri[i]].wire.x << " " << this->flock[nOri[i]].wire.t << endl;
 		// double dx =  this->wire.x - this->flock[nOri[i]].wire.x;
 		// double dy =  this->wire.y - this->flock[nOri[i]].wire.y;
 		// double d = sqrt(pow(dx,2) + pow(dy,2));
 		double th = this->flock->at(nOri[i]).wire.t*PI/180;
 		// cout << " th: " << th;
		sum += th;
		cnt++;
 	}
 	if (sum != 0)
 		oriH = sum / cnt;

 	// cout << "sum " << sum << " " << oriH << endl;


  	double atrX = 0;
 	double atrY = 0;
 	for (int i = 0; i < this->nAtr.size(); i++){
 		double dx = this->flock->at(nAtr[i]).wire.x - this->wire.x;
 		double dy = this->flock->at(nAtr[i]).wire.y - this->wire.y;
 		double d = sqrt(pow(dx,2) + pow(dy,2));
		atrX += dx/d/d;	
		atrY += dy/d/d;	
 	}

 	// cout << "Att Vel: " <<  atrX <<  " " << atrY <<  " " <<  atan2(atrY, atrX) << endl;
 	// return (atan2(atrY, atrX) + atan2(repY, repX) + oriH) / 3.0;
 	// return atan2(atrY + repY, atrX + repX);
 	// cout >> oriH >> endl;

 	// if (repY == 0 && repX == 0)
 	// 	return wire.t*PI/180;
 	// else
 	//  	return atan2(repY, repX);

 	return (atan2(repY, repX) + atan2(atrY, atrX)) / 2
 }

 double Robot::wallRepulsion(double xlim, double ylim){
 	// Returns heading based on wall repulsions
	double wallX = 0;
 	double wallY = 0;
 	bool noWalls = true;
 	if (wire.x >= xlim){
 		noWalls = false;
 		wallX = xlim - wire.x;
 	}
 	if (wire.x <= -xlim){
 		noWalls = false;
 		wallX = xlim - wire.x;
 	}
 	if (wire.y >= ylim){
 		noWalls = false;
 		wallY = ylim - wire.y;
 	}
 	if (wire.y <= -ylim){
 		noWalls = false;
 		wallY = ylim - wire.y;
 	}
 	if (noWalls) 
 		return NAN;
 	// cout << "My XY: "<< wire.x << " " << wire.y << " MY HEADING: " << wire.t << "Desired: " << atan2(wallY, wallX) << endl;
 	return atan2(wallY, wallX);
 }

 double Robot::distanceToPoint(double x, double y){
 	return sqrt(pow(this->wire.x - x, 2) + pow(this->wire.y - y, 2));
 }

 double Robot::distanceToRobot(vector<Robot> *flock, int id){
 	return this->distanceToPoint(flock->at(id).wire.x, flock->at(id).wire.y);
 }

 double Robot::distanceToRobot(Robot robot){
 	return this->distanceToPoint(robot.wire.x, robot.wire.y);
 }

void Robot::render(){
	wire.render(1,selected);
};

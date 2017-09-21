#include "Robot.hpp"

Robot::Robot(){
	this->vel = 0;
	this->lx = 0;
	this->ly = 0;
	this->lt = 0;
	this->oy = 0;
	this->ot = 0;
	this->ot = 0;
	this->vx = 0;
	this->vy = 0;
	this->vt = 0;
	this->selected = false;
};

Robot::~Robot(){};

int Robot::init(double x,double y,double w,double h,double t,double vel,char * filename){
	tex.x = x;
	tex.y = y;
	tex.w = w;
	tex.h = h;
	tex.t = t;
	this->vel = vel;
	return tex.init(x,y,w,h,t,filename);
};

bool Robot::respawn(double x,double y){
	tex.x = x;
	tex.y = y;
};

void Robot::setRef(double gx,double gy){
	this->lx = gx;
	this->ly = gy;
}

void Robot::update(bool col){

	// Distance to target
	double dlx = lx - tex.x;
	double dly = ly - tex.y;
	double dl = sqrt(pow(dlx,2)+pow(dly,2));

	// Direction to target
	double rx = dlx/dl;
	double ry = dly/dl;

	// Stop if target was reached
	if(dl < DEFAULT_TOL*tex.w){
		rx = 0;
		ry = 0;		
	}

	// Col check
	if(col){
		tex.x = ox;
		tex.y = oy;
	}else{
		ox = tex.x;
		oy = tex.y;
	}

	vx = vel*rx;
	vy = vel*ry;

	tex.x += vx;
	tex.y += vy;

};

void Robot::render(){
	tex.render(1,selected);
};

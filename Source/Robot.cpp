#include "Robot.hpp"

Robot::Robot(){
	this->life = 1.0;
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

	// Path vector
	this->xP = new vector<double>;
	this->yP = new vector<double>;
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
	xP->insert(xP->begin(),gx);
	yP->insert(yP->begin(),gy);
	updateVector();
}

int Robot::updateVector(){
	if(xP->size() <= 0)
		return 0;
	lx = xP->back();
	ly = yP->back();
	xP->pop_back();
	yP->pop_back();
	return 1;
}

void Robot::update(bool col){

	if(life >= 1)
		life = 1;

	// Distance to local target
	double dlx = lx - tex.x;
	double dly = ly - tex.y;
	double dl = sqrt(pow(dlx,2)+pow(dly,2));

	// Direction to target
	double rx = dlx/dl;
	double ry = dly/dl;

	// Check if target was reached
	if(dl < DEFAULT_TOL*tex.w)
		if(!updateVector()){
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
	if(selected)
		renderPath();
	tex.render(1,selected);

	// Life bar render
	double lifeScale = 0.15+0.35*life;
	glColor4f(0.0,0.0,0.0,0.75);
	glPushMatrix();
		glTranslatef(-tex.x,-tex.y,0);
		glScalef(tex.w,tex.h,1);			
		glPushMatrix();
			for(int i = 0; i < 4; i++){
				glRotatef(90,0,0,1);
				glBegin(GL_QUADS);
					glVertex2f(-0.05,lifeScale);
					glVertex2f(-0.05,0.50);
					glVertex2f(+0.05,0.50);
					glVertex2f(+0.05,lifeScale);
				glEnd();
			}
		glPopMatrix();
	glPopMatrix();
};


void Robot::renderPath(){
	for(int i = 0; i < xP->size(); i++){
		glColor4f(1,0,0,0.75);
		glPushMatrix();
			glTranslatef(-xP->at(i),-yP->at(i),0);
			glScalef(tex.w/4,tex.h/4,1);
			glRotatef(45,0,0,1);
			glTranslatef(-0.5,-0.5,0);
			glBegin(GL_QUADS);
				glVertex2f(0,0);
				glVertex2f(1,0);
				glVertex2f(1,1);
				glVertex2f(0,1);
			glEnd();
		glPopMatrix();
	}
};

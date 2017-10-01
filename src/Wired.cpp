#include "Wired.hpp"

Wired::Wired(double x,double y,double w,double h,double t,vector<pair<double, double>> shape){
	this->x = x;
	this->y = y;
	this->w = w;
	this->h = h;
	this->t = t;
	this->shape = shape;
};

Wired::~Wired(){};

void Wired::render(bool global,bool highlight, float r=1.0, float g=1.0, float b=1.0){
	glPushMatrix();
		if(highlight)
			glLineWidth(3);
		else
			glLineWidth(1);
		glColor3f(r,g,b);
		if(global){
			glTranslatef(-x,-y,0);
			glScalef(w,h,1);
		}
		else{
			glTranslatef(x,y,0);
			glScalef(w,-h,1);
		}
		glRotatef(t,0,0,1);
		glTranslatef(-0.5,-0.5,0);
		glBegin(GL_LINE_LOOP);
			for(auto p : this->shape)
				glVertex2f(p.first,p.second);
		glEnd();
		glColor3f(1.0,1.0,1.0);
		glLineWidth(1);
	glPopMatrix();
};

#ifndef ROBOT_H
#define ROBOT_H
#include "includes.hpp"
#include "Textured.hpp"

class Robot{

	private:

		double vel;

		double lx;
		double ly;
		double lt;

		double ox;
		double oy;
		double ot;

		double vx;
		double vy;
		double vt;

	public:

		double life;
		bool selected;
		Textured tex;

		// Path vector
		vector<double> *xP;
		vector<double> *yP;

		Robot();
		virtual ~Robot();
		int init(double x,double y,double w,double h,double r,double vel,char * filename);
		bool respawn(double x,double y);
		void setRef(double gx,double gy);
		void update(bool col);
		void render();
};

#endif

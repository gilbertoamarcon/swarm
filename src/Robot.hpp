#ifndef ROBOT_H
#define ROBOT_H
#include "includes.hpp"
#include "Wired.hpp"

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

		bool selected;
		Wired wire;

		Robot();
		virtual ~Robot();
		int init(double x,double y,double w,double h,double r,double vel,vector<pair<double, double>> shape);
		bool respawn(double x,double y);
		void setRef(double gx,double gy);
		void update(bool col);
		void render();
};

#endif

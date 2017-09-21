#ifndef ROBOT_H
#define ROBOT_H
#include "includes.hpp"
#include "Wired.hpp"

/*
Robot representation.
Robot are represented by a wire-shaped object called 'wire'.
*/

class Robot{

	private:

		double vel;	// Absolute linear velocity

		// Goal Target Position/Angle
		double lx;
		double ly;
		double lt;

		// Old(Previous) Robot position/angle (for collision enforcement)
		double ox;
		double oy;
		double ot;

		// Linear(in Cartesian coordinates)/angular velocity
		double vx;
		double vy;
		double vt;

	public:

		bool selected; // User selected (my mouse selection)
		Wired wire;		// Visual representation  

		Robot();
		virtual ~Robot();
		int init(double x,double y,double w,double h,double r,double vel,vector<pair<double, double>> shape);
		bool respawn(double x,double y);
		void setGoalTargetPos(double gx,double gy);
		void update(bool col);
		void render();
};

#endif

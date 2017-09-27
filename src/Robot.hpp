#ifndef ROBOT_H
#define ROBOT_H
#include "includes.hpp"
#include "Wired.hpp"

/*
Robot representation.
Robot are represented by a wire-shaped object called 'wire'.
*/

class Robot{

	//private:
	public:
		int id; 	// Robot ID
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

		// Reynold's Radii
		double rRep;
		double rOri;
		double rAtr;

		// Neighbor Sets
		vector<int> nRep;
		vector<int> nOri;
		vector<int> nAtr;

		// Vector of Neighbor IDs
		vector <int> nbors;
		// Vector of all robots in the swarm
		vector <Robot> *flock;

	// public:

		bool selected; // User selected (my mouse selection)
		Wired wire;		// Visual representation  

		Robot();
		Robot(int id);
		virtual ~Robot();
		int init(double x,double y,double w,double h,double r,double vel,vector<pair<double, double>> shape);
		void respawn(double x,double y);
		void setGoalTargetPos(double gx,double gy);
		void update(bool col);
		void updateNeighbors();
		void render();
		vector<int> getNeighbors(double radius);
		double swarm();
		double reynoldsRules();
		double wallRepulsion(double xlim, double ylim);
		double distanceToPoint(double x, double y);
		double distanceToRobot(vector<Robot> *flock, int id);
		double distanceToRobot(Robot robot);
};

#endif

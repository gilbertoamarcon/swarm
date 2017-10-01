#ifndef ROBOT_H
#define ROBOT_H
#include "includes.hpp"
#include "Wired.hpp"

/*
Robot representation.
Robot are represented by a wire-shaped object called 'wire'.
*/

class Robot: public Wired{

	private:
		double vel;	// Absolute linear velocity
		double steer; // Absolute angular velocity

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

		// Neighbor Sets
		vector<int> neighbor_rep;
		vector<int> neighbor_ori;
		vector<int> neighbor_att;

		// Vector of Neighbor IDs
		vector <int> nbors;

	public:
		int id; 	// Robot ID

		// Reynold's Radii
		double radius_rep;
		double radius_ori;
		double radius_att;

		// Vector of all robots in the swarm
		vector <Robot> *flock;

		bool selected; // User selected (my mouse selection)

		Robot();
		Robot(int id);
		virtual ~Robot();
		int init(
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
				);
		void respawn(double x,double y);
		void setGoalTargetPos(double gx,double gy);
		void update();
		void updateNeighbors();
		void render_robot();
		vector<int> getNeighbors(double radiusMax, double radiusMin = 0.0);
		double swarm();
		double reynoldsRules();
		double wallRepulsion(double xlim, double ylim);
		double distanceToPoint(double x, double y);
		double distanceToRobot(vector<Robot> *flock, int id);
		double distanceToRobot(Robot robot);
		bool checkCol();
};

#endif

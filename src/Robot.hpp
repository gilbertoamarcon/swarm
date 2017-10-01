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
		vector<Robot*> neighbor_rep;
		vector<Robot*> neighbor_ori;
		vector<Robot*> neighbor_att;

	public:

		// Reynold's Radii
		double radius_rep;
		double radius_ori;
		double radius_att;

		// Vector of all robots in the swarm
		vector <Robot> *flock;

		bool selected; // User selected (my mouse selection)

		Robot();
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
		void set_goal_target_pos(double gx,double gy);
		void update();
		void update_neighbors();
		vector<Robot*> get_neighbors(double radiusMax, double radiusMin = 0.0);
		double swarm();
		double reynolds_rules();
		double wall_repulsion(double xlim, double ylim);

		// Distances
		double distance_to_point(double x, double y);
		double distance_to_robot(Robot *robot);

		bool check_col();
		void render_robot();
};

#endif

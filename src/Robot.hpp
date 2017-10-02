#ifndef ROBOT_H
#define ROBOT_H
#include "includes.hpp"
#include "Wired.hpp"
#include "mlp.hpp"

/*
Robot representation.
*/

class Robot: public Wired{

	private:
		double v; // Absolute linear velocity
		double a; // Absolute angular velocity

		// Goal Target Position
		pair<double,double> goal;

		// Reynold's Radii
		double radius_rep;
		double radius_ori;
		double radius_att;

		// Vector of all robots in the swarm
		vector<Robot> *flock;

		// Neighbor Sets
		set<Robot*> neighbor_rep;
		set<Robot*> neighbor_ori;
		set<Robot*> neighbor_att;

	public:

		Mlp *mlp;

		bool selected; // User selected (my mouse selection)
		bool leader;
		double acc_dist;

		Robot(
				double x,
				double y,
				double w,
				double h,
				double r,
				double v,
				double s,
				vector<pair<double, double>> shape,
				vector<Robot> *flock,
				double radius_rep,
				double radius_ori,
				double radius_att,
				bool leader
			);
		virtual ~Robot();
		void respawn(double x,double y,double t);
		void set_goal_target_pos(double gx,double gy);
		void update();
		void update_neighbors();
		set<Robot*> get_neighbors(double radiusMax, double radiusMin = 0.0);
		double swarm();
		pair<double,double> compute_force(set<Robot*> &neighbors);
		pair<double,double> compute_centroid(set<Robot*> &neighbors);
		double leader_reasoning();
		double reynolds_rules();
		double wall_repulsion(double xlim, double ylim);

		// Distances
		double angle_to_point(pair<double,double> &input);
		double distance_to_point(pair<double,double> &input);
		double distance_to_point(double x, double y);
		double distance_to_robot(Robot *robot);

		bool check_col();
		void render_robot();
};

#endif

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
		pair<double,double> neighbor_centroid;

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
		vector<pair<double, double>> prevCoords;
		int goal_group;
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
				bool leader,
				int goal_group
			);
		virtual ~Robot();
		void respawn(double x,double y,double t,Mlp *mlp);
		void set_goal_target_pos(double gx,double gy);
		void update(double weight, vector<pair<int, int>> &goals);
		void update_neighbors();
		set<Robot*> get_neighbors_M(double radiusMax, double radiusMin = 0.0);
		set<Robot*> get_neighbors_V(double radiusMax, double radiusMin = 0.0);
		set<Robot*> get_k_nearest(set<Robot*>, int);
		double swarm();
		pair<double,double> compute_force(set<Robot*> &neighbors);
		pair<double,double> compute_centroid(set<Robot*> &neighbors, pair<double,double> prev);
		double leader_reasoning();
		double reynolds_rules();
		double wall_repulsion(double xlim, double ylim);
		void update_trail();
		// Distances
		double angle_to_point(pair<double,double> &input);
		double Robot::angle_to_point(double x, double y);
		double distance_to_point(pair<double,double> &input);
		double distance_to_point(double x, double y);
		double distance_to_robot(Robot *robot);
		double sq_distance_to_point(pair<double,double> &input);
		double sq_distance_to_point(double x, double y);
		double sq_distance_to_robot(Robot *robot);
		double sq_distance_to_closest_goal(vector<pair<int,int>> &input);

		bool check_col();
		void render_robot();
};

#endif

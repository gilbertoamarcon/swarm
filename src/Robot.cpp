#include "Robot.hpp"
#include <cmath>
#include <iostream>
using namespace std;

Robot::Robot(
				double x,
				double y,
				double w,
				double h,
				double r,
				double v,
				double a,
				vector<pair<double, double>> shape,
				vector<Robot> *flock,
				double radius_rep,
				double radius_ori,
				double radius_att,
				bool leader
			):Wired(x,y,w,h,r,shape){
	this->v					= v;
	this->a					= a;
	goal.first				= 0;
	goal.second				= 0;
	this->radius_rep		= radius_rep;
	this->radius_ori		= radius_ori;
	this->radius_att		= radius_att;
	this->flock				= flock;
	this->leader			= leader;
	this->selected			= true;
	this->acc_dist			= 0.0;
	this->mlp				= NULL;
	this->goal				= pair<double,double>(0.0,0.0);
	this->neighbor_centroid	= pair<double,double>(0.0,0.0);

	#if ENABLE_TRAIL
		this->prevCoords 	= {};	for (int i=0; i<TRAIL_LENGTH; i++) prevCoords.push_back(pair<double, double>(x, y));
	#endif
}

Robot::~Robot(){};

void Robot::respawn(double x,double y,double t,Mlp *mlp){
	this->x				= x;
	this->y				= y;
	this->t				= t;
	this->acc_dist		= 0.0;
	this->mlp			= mlp;
	this->selected 		= true;
	#if ENABLE_TRAIL
		this->prevCoords 	= {};	for (int i=0; i<TRAIL_LENGTH; i++) prevCoords.push_back(pair<double, double>(x, y));
	#endif
}

void Robot::set_goal_target_pos(double gx,double gy){
	this->goal.first	= gx;
	this->goal.second	= gy;
}

void Robot::update(double weight){
	#if ENABLE_TRAIL 
		update_trail();
	#endif

	acc_dist += weight*distance_to_point(goal);

	double goal_t = this->t;
	if(leader)
		goal_t = leader_reasoning();
	else
		goal_t = swarm();

	// Update heading and velocities
	double delta = goal_t-this->t;
	angle_wrap(delta);

	this->t += this->a*delta;
	angle_wrap(this->t);

	// Send velocity commands
	this->x += v*cos(deg_to_rad(this->t));
	this->y += v*sin(deg_to_rad(this->t));

};

void Robot::update_trail(){
		for (int i=0; i<prevCoords.size()-1; i++){
			prevCoords[i].first = prevCoords[i+1].first;
			prevCoords[i].second = prevCoords[i+1].second;
		}
		prevCoords.back() = {x, y};
}

// Get agents between radii
set<Robot*> Robot::get_neighbors(double radiusMax, double radiusMin = 0.0){
	set<Robot*> nbors;
	for(auto &r : *flock){
		double d = distance_to_robot(&r);
		if(this != &r && d <= radiusMax && d >= radiusMin)
			nbors.insert(&r);
	}
	return nbors;
}


// Update flocking neighbors
 void Robot::update_neighbors(){
	this->neighbor_rep = this->get_neighbors(this->radius_rep);
	this->neighbor_ori = this->get_neighbors(this->radius_ori);
	this->neighbor_att = this->get_neighbors(this->radius_att, this->radius_rep);
}

// Returns next heading (in radians) based on local interactions
double Robot::swarm(){
	double w_rep = this->wall_repulsion(WORLD_SIZE_X, WORLD_SIZE_Y);
	if(w_rep == w_rep)
		return w_rep;
	else
		return this->reynolds_rules();
}

pair<double, double> Robot::compute_force(set<Robot*> &neighbors){
	pair<double,double> force(0.0,0.0);
	for(auto &r : neighbors){
		double dx = r->x - this->x;
		double dy = r->y - this->y;
		double d2 = pow(dx,2) + pow(dy,2);
		force.first  += dx/d2;
		force.second += dy/d2;
	}
	return force;
}

pair<double, double> Robot::compute_centroid(set<Robot*> &neighbors, pair<double,double> prev){
	if(neighbors.size() == 0)
		return prev;
	pair<double,double> centroid(0.0,0.0);
	for(auto &r : neighbors){
		centroid.first  += r->x;
		centroid.second += r->y;
	}
	centroid.first  /= neighbors.size();
	centroid.second /= neighbors.size();
	return centroid;
}

double Robot::leader_reasoning(){

	// Leader neighbors
	set<Robot*> neighbor_leader;
	for(auto &r : *flock)
		if(!(r.leader))
			neighbor_leader.insert(&r);

	// Neighbor centroid
	neighbor_centroid	= compute_centroid(neighbor_leader,pair<double,double>(this->x,this->y));

	// Distance to centroids
	double distance_to_neighbor_centroid	= distance_to_point(neighbor_centroid);
	double distance_to_goal					= distance_to_point(goal);

	// Angle to centroids
	double angle_to_neighbor_centroid		= rad_to_deg(angle_to_point(neighbor_centroid))	- this->t;
	double angle_to_goal					= rad_to_deg(angle_to_point(goal))				- this->t;
	angle_wrap(angle_to_neighbor_centroid);
	angle_wrap(angle_to_goal);

	// Loading inputs
	mlp->x[0] = deg_to_rad(angle_to_goal);
	mlp->x[1] = deg_to_rad(angle_to_neighbor_centroid);
	mlp->x[2] = distance_to_goal/WORLD_SIZE_X;
	mlp->x[3] = distance_to_neighbor_centroid/WORLD_SIZE_X;
	mlp->eval();
	double goal_direction = rad_to_deg(2*mlp->o[0]);

	// double goal_direction = angle_to_goal;

	return goal_direction + this->t;
}

// Returns desired heading based on swarming rules
double Robot::reynolds_rules(){

	// Update neighbor sets for radius_rep, radius_ori, radius_att
	update_neighbors();

	// Repulsion Vector
	pair<double,double> rep = compute_force(neighbor_rep);

	// Attraction Vector
	pair<double,double> att = compute_force(neighbor_att);

	// Orientation Vector
	pair<double,double> ori(0.0,0.0);
	for(auto &r : neighbor_ori){
		double d = distance_to_robot(r);
		double th = deg_to_rad(r->t);
		ori.first  += cos(th)/d;
		ori.second += sin(th)/d;
	}

	// Add up all velocities, normalized and weighted by distance
	double x = -rep.first  +att.first  +ori.first;
	double y = -rep.second +att.second +ori.second;

	// Go straight in the absence of neighbors
	if(x == 0 && y == 0)
		return this->t;
	else
		return rad_to_deg(atan2(y, x));
}

// Returns heading based on wall repulsions
double Robot::wall_repulsion(double xlim, double ylim){
	if(abs(x) >= xlim) return rad_to_deg(atan2(0.0, xlim-x));
	if(abs(y) >= ylim) return rad_to_deg(atan2(ylim-y, 0.0));
	return NAN;
}

double Robot::angle_to_point(pair<double,double> &input){
	return atan2(input.second-this->y, input.first-this->x);
}

double Robot::distance_to_point(pair<double,double> &input){
	return sqrt(pow(this->x-input.first, 2) + pow(this->y-input.second, 2));
}

double Robot::distance_to_point(double x, double y){
	return sqrt(pow(this->x-x, 2) + pow(this->y-y, 2));
}

double Robot::distance_to_robot(Robot *robot){
	return this->distance_to_point(robot->x, robot->y);
}

 bool Robot::check_col(){
	for(auto const &r : *flock)
		if(this != &r){
			if(this->x+this->w/2 < r.x-r.w/2) continue;
			if(this->x-this->w/2 > r.x+r.w/2) continue;
			if(this->y+this->h/2 < r.y-r.h/2) continue;
			if(this->y-this->h/2 > r.y+r.h/2) continue;
			return true;
		}	
	return false;
}

void Robot::render_robot(){
	if(leader)
		render(1,selected,1.0,0.0,0.0);
	else
		render(1,selected);
};

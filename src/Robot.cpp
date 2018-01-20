#include "Robot.hpp"
#include <float.h>
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
				bool leader,
				int goal_group
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
	this->goal_group 		= goal_group;

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

bool Robot::is_within_goal_radius(vector<Textured> &goals){
	if (sq_distance_to_closest_object(goals) < GOAL_RADIUS_SQ)
		return true;
	else
		return false;
}

void Robot::update(double weight, vector<Textured> &goals, vector<Textured> &obstacles){
	#if ENABLE_TRAIL 
		update_trail();
	#endif
	#if CLASSIC_REW
		acc_dist += weight*sqrt(sq_distance_to_closest_object(goals));
	#endif
	#if ALTERNATE_REW
		double d = distance_to_point(goal);
		acc_dist = (d > acc_dist && acc_dist!=0) ? acc_dist : d;
	#endif

	double goal_t = this->t;
	if(leader){
		// Get desired vel and dir
		pair <double, double> command = leader_reasoning(obstacles);
		this-> v += 0.1*(command.first - this->v);
		goal_t = (1 - SWARM_PULL)*command.second + SWARM_PULL*swarm();
	}
	else
		goal_t = swarm();

	// Update heading and velocities
	double delta = goal_t-this->t;
	angle_wrap(delta);

	this->t += this->a*delta;
	angle_wrap(this->t);

	// Send velocity commands
	float dx = v*cos(deg_to_rad(this->t));
	float dy = v*sin(deg_to_rad(this->t));
	if (!this->check_col(obstacles, dx, dy)){
		this->x += dx;
		this->y += dy;
	}
};

void Robot::update_trail(){
	for (int i=0; i<prevCoords.size()-1; i++){
		prevCoords[i].first = prevCoords[i+1].first;
		prevCoords[i].second = prevCoords[i+1].second;
	}
	prevCoords.back() = {x, y};
}

// Get agents between radii
set<Robot*> Robot::get_neighbors_M(double radiusMax, double radiusMin = 0.0){
	set<Robot*> nbors;
	for(auto &r : *flock){
		if(this != &r){
			double d = sq_distance_to_robot(&r);
			if (d <= radiusMax && d >= radiusMin){
				nbors.insert(&r);
			}
		}
	}
	return nbors;
}

set<Robot*> Robot::get_neighbors_V(double radiusMax, double radiusMin = 0.0){
	set<Robot*> nbors;
	vector<pair<Robot*, pair<double, double>>> candidates;
	// assume that all agents are the same size and take maximum dimension as "diameter"
	double agent_radius = max(this->h, this->w) / 2;
	for(auto &r : *flock){
		double d = sq_distance_to_robot(&r);
		if(this != &r && d <= radiusMax && d >= radiusMin){
			pair<double, double> rxy(r.x, r.y);
			double angle = rad_to_deg(angle_to_point(rxy)) - this->t;
 			angle_wrap(angle);
 			angle = deg_to_rad(angle);
 			d = sqrt(d); // need actual distance (not squared distance) for trig
 			if (angle > -1*VIS_ANGLE && angle < VIS_ANGLE) {
 				// check other candidate neighbors to figure out whether this agent occludes/is occluded
 				bool occluded = false;
 				for (int i = 0; i < candidates.size(); i++){
 					double other_angle = candidates[i].second.second;
 					double other_d = candidates[i].second.first;
 					if (other_d > d){ // other is farther away
 						double occ_angle = atan2(agent_radius, d);
 						if (abs(other_angle - angle) < 2*abs(occ_angle)){
 							// other is occluded by this one, so remove other
 							candidates.erase(candidates.begin()+i);
 						}
 					}
 					else { // this one is farther away
 					 	double occ_angle = atan2(agent_radius, other_d);
 						if (abs(other_angle - angle) < 2*abs(occ_angle)){
 							// this one is occluded by another, so don't add it
 							occluded = true;
 							break;
 						}	
 					}
 				}
 				if (!occluded){
 					pair<double, double> loc(d, angle);
					pair<Robot*, pair<double, double>> agent(&r, loc);
					candidates.push_back(agent);
 				}
			}
		}
	}
	// add all remaining candidates to the set
	for (auto &agent : candidates){
		nbors.insert(agent.first);
	}

	return nbors;
}

// vector<Robot*> Robot::get_neighbors_T(int nTop, double radiusMax, double radiusMin = 0.0){
// 	vector<Robot*> nbors;
// 	for(auto &r : *flock){
// 		double d = distance_to_robot(&r);
// 		if(this != &r && d <= radiusMax && d >= radiusMin)
// 			nbors.push_back(&r);
// 	}
// 	sort(nbors.begin(), nbors.end(), [this](Robot* r1, Robot* r2) {
// 		return r1->distance_to_robot(this) < r2->distance_to_robot(this);
// 	});
// 	// set<Robot*> nborset;
// 	// for(int i=0; i<nbors.size() && i<nTop ; i++)
// 	// 	nborset.insert(nbors.at(i));
// 	return nbors;
// }

set<Robot*> Robot::get_k_nearest(set<Robot*> nbors, int k){
	vector<Robot*> nborsv;
	for(auto &r : nbors) 
		nborsv.push_back(r);
	sort(nborsv.begin(), nborsv.end(), [this](Robot* r1, Robot* r2) {
		return r1->distance_to_robot(this) < r2->distance_to_robot(this);
	});
	set<Robot*> nborset;
	for(int i=0; i<nborsv.size() && i<k ; i++)
		nborset.insert(nborsv.at(i));
	return nborset;
}


// Update flocking neighbors
 void Robot::update_neighbors(){
 	if (comm_model == 'T' || comm_model == 'M'){
		this->neighbor_rep = this->get_neighbors_M(this->radius_rep);
		this->neighbor_ori = this->get_neighbors_M(this->radius_ori);
		this->neighbor_att = this->get_neighbors_M(this->radius_att, this->radius_rep);
		if (comm_model == 'T'){
			this->neighbor_rep = get_k_nearest(this->neighbor_rep, N_TOP);
			this->neighbor_ori = get_k_nearest(this->neighbor_ori, N_TOP);
			this->neighbor_att = get_k_nearest(this->neighbor_att, N_TOP-this->neighbor_rep.size());
		}
	}
	else if (comm_model == 'V'){
		this->neighbor_rep = this->get_neighbors_V(this->radius_rep);
		this->neighbor_ori = this->get_neighbors_V(this->radius_ori);
		this->neighbor_att = this->get_neighbors_V(this->radius_att, this->radius_rep);
	}
}

// Returns next heading (in radians) based on local interactions
double Robot::swarm(){
	#ifdef BOUNDED_WORLD
		return this->reynolds_rules();
	#else
		double w_rep = this->wall_repulsion(WORLD_SIZE_X, WORLD_SIZE_Y);
		if(w_rep == w_rep)
			return w_rep;
		else
			return this->reynolds_rules();
	#endif
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

pair<double, double> Robot::leader_reasoning(vector<Textured> &obstacles){
	#if LEARNING
		// Non-Leader neighbors
		set<Robot*> neighbor_leader;
		for(auto &r : *flock)
			// if(!(r.leader) && find(neighbor_att.begin(), neighbor_att.end(), &r) != neighbor_att.end())
			if(find(neighbor_att.begin(), neighbor_att.end(), &r) != neighbor_att.end())
				neighbor_leader.insert(&r);

		// Centroid of non-leaders
		neighbor_centroid	= compute_centroid(neighbor_leader,pair<double,double>(this->x,this->y));

		// Closest obstacle
		Textured *obstacle = closest_object(obstacles);

		// Distance to centroids
		double distance_to_neighbor_centroid	= distance_to_point(neighbor_centroid);
		double distance_to_goal					= distance_to_point(goal);
		double distance_to_obstacle				= 0;
		if (obstacle)
			distance_to_obstacle				= distance_to_point(obstacle->x, obstacle->y);

		// Angle to centroids
		double angle_to_neighbor_centroid		= rad_to_deg(angle_to_point(neighbor_centroid))	- this->t;
		double angle_to_goal					= rad_to_deg(angle_to_point(goal))				- this->t;
		double angle_to_obstacle				= 0;
		if (obstacle){
			angle_to_obstacle 					= rad_to_deg(angle_to_point(obstacle->x, obstacle->y)) - this->t;
			angle_wrap(angle_to_obstacle);
		}
		angle_wrap(angle_to_neighbor_centroid);
		angle_wrap(angle_to_goal);

		// Loading inputs
		if (NUM_OBSTACLES != 0){
			mlp->x[0] = deg_to_rad(angle_to_goal);
			mlp->x[1] = deg_to_rad(angle_to_neighbor_centroid);
			mlp->x[2] = deg_to_rad(angle_to_obstacle);
			mlp->x[3] = distance_to_goal/WORLD_SIZE_X;
			mlp->x[4] = distance_to_neighbor_centroid/WORLD_SIZE_X;
			mlp->x[5] = distance_to_obstacle/WORLD_SIZE_X;
		}
		else {
			mlp->x[0] = deg_to_rad(angle_to_goal);
			mlp->x[1] = deg_to_rad(angle_to_neighbor_centroid);
			mlp->x[2] = distance_to_goal/WORLD_SIZE_X;
			mlp->x[3] = distance_to_neighbor_centroid/WORLD_SIZE_X;
		}
		mlp->eval();
		// 1st MLP output denotes direction
		double desired_dir = rad_to_deg(2*mlp->o[0]);
		// 2nd MLP output denotes speed
		double desired_vel = map_range((mlp->o[1]), -PI/2, PI/2, 0, ROBOT_VEL*2);
	#else
		double angle_to_goal					= rad_to_deg(angle_to_point(goal))				- this->t;
		double desired_dir = angle_to_goal;
	#endif
		pair<double,double> command = make_pair(desired_vel, desired_dir + this->t);
	return command;
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

	// Velocity Matching
	double vel = this->v;
	if (neighbor_ori.size()){
		double sum = 0;
		for(auto &r : neighbor_ori){
			// double d = distance_to_robot(r);
			sum += r->v;
		}
		// cout << sum/neighbor_ori.size() << endl;
		this->v = sum/neighbor_ori.size();
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

double Robot::angle_to_point(double x, double y){
	return atan2(y-this->y, x-this->x);
}

double Robot::sq_distance_to_point(pair<double,double> &input){
	return pow(this->x-input.first, 2) + pow(this->y-input.second, 2);
}

double Robot::sq_distance_to_closest_object(vector<Textured> &objects){
	
	double min_distance = DBL_MAX;
	for (auto &o : objects)
	{
		double distance = pow(this->x-o.x, 2) + pow(this->y-o.y, 2);
		if (distance < min_distance)
			min_distance = distance;
	}
	return min_distance;
}

Textured* Robot::closest_object(vector<Textured> &objects){
	
	double min_distance = DBL_MAX;
	Textured* closest = NULL;
	for (auto &o : objects)
	{
		double distance = pow(this->x-o.x, 2) + pow(this->y-o.y, 2);
		if (distance < min_distance){
			min_distance = distance;
			closest = &o;
		}
	}
	return closest;
}

double Robot::sq_distance_to_point(double x, double y){
	return pow(this->x-x, 2) + pow(this->y-y, 2);
}

double Robot::sq_distance_to_robot(Robot *robot){
	return this->sq_distance_to_point(robot->x, robot->y);
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

bool Robot::check_col(vector<Textured> &obstacles, float dx, float dy){
	for(auto const &o : obstacles){
			if(this->x+this->w/2+dx < o.x-o.w/2) continue;
			if(this->x-this->w/2+dx > o.x+o.w/2) continue;
			if(this->y+this->h/2+dy < o.y-o.h/2) continue;
			if(this->y-this->h/2+dy > o.y+o.h/2) continue;
			return true;
	}
	return false;
}

void Robot::render_robot(){
	if(leader)
		render(1,selected,1.0,0.0,0.0);
	else
		render(1,selected,1.0,1.0,1.0);
};

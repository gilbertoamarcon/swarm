#include "Robot.hpp"

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

void Robot::update(double weight, vector<pair<int, int>> &goals){
	#if ENABLE_TRAIL 
		update_trail();
	#endif
	#if CLASSIC_REW
		acc_dist += weight*sqrt(sq_distance_to_closest_goal(goals));
	#endif
	#if ALTERNATE_REW
		double d = distance_to_point(goal);
		acc_dist = (d > acc_dist && acc_dist!=0) ? acc_dist : d;
	#endif

	double goal_t = this->t;
	if(leader)
		goal_t = (1 - SWARM_PULL)*leader_reasoning() + SWARM_PULL*swarm();
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
	for(auto &r : *flock){
		double d = distance_to_robot(&r);
		if(this != &r && d <= radiusMax && d >= radiusMin){
			pair<double, double> rxy(r.x, r.y);
			double angle = rad_to_deg(angle_to_point(rxy)) - this->t;
 			angle_wrap(angle);
 			angle = deg_to_rad(angle);
 			if (angle > -1*VIS_ANGLE && angle < VIS_ANGLE)
				nbors.insert(&r);
		}
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
 	if (COMM_MODEL == 'T' || COMM_MODEL == 'M'){
		this->neighbor_rep = this->get_neighbors_M(this->radius_rep);
		this->neighbor_ori = this->get_neighbors_M(this->radius_ori);
		this->neighbor_att = this->get_neighbors_M(this->radius_att, this->radius_rep);
		if (COMM_MODEL == 'T'){
			this->neighbor_rep = get_k_nearest(this->neighbor_rep, N_TOP);
			this->neighbor_ori = get_k_nearest(this->neighbor_ori, N_TOP);
			this->neighbor_att = get_k_nearest(this->neighbor_att, N_TOP-this->neighbor_rep.size());
		}
	}
	else if (COMM_MODEL == 'V'){
		this->neighbor_rep = this->get_neighbors_V(this->radius_rep);
		this->neighbor_ori = this->get_neighbors_V(this->radius_ori);
		this->neighbor_att = this->get_neighbors_V(this->radius_att, this->radius_rep);
	}
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
	#if LEARNING
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
	#else
		double angle_to_goal					= rad_to_deg(angle_to_point(goal))				- this->t;
		double goal_direction = angle_to_goal;
	#endif

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

double Robot::angle_to_point(double x, double y){
	return atan2(y-this->y, x-this->x);
}

double Robot::sq_distance_to_point(pair<double,double> &input){
	return pow(this->x-input.first, 2) + pow(this->y-input.second, 2);
}

double Robot::sq_distance_to_closest_goal(vector<pair<int,int>> &input){
	
	double min_distance = DBL_MAX;
	for (auto &goal : input)
	{
		double distance = pow(this->x-goal.first, 2) + pow(this->y-goal.second, 2);
		if (distance < min_distance)
			min_distance = distance;
	}
	return min_distance;
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

void Robot::render_robot(){
	if(leader)
		render(1,selected,1.0,0.0,0.0);
	else
		render(1,selected);
};

#ifndef WIRED_H
#define WIRED_H
#include "includes.hpp"

class Wired{
	public:
		double x;
		double y;
		double w;
		double h;
		double t;
		vector<pair<double, double>> shape;
		Wired();
		virtual ~Wired();
		void update(double x,double y,double w,double h,double r);
		void render(bool global,bool highlight);
		int init(double x,double y,double w,double h,double r,vector<pair<double, double>> shape);
};

#endif

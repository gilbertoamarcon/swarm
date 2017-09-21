#ifndef WIRED_H
#define WIRED_H
#include "includes.hpp"

/*
Wired-Shaped object representation.
Wire-shaped objects are defined by a list of points representing a polygon.
*/

class Wired{
	public:
		double x;	// Position coordinate x
		double y;	// Position coordinate y
		double t;	// Object heading theta (in Degrees)
		double w;	// Object width (for collision detection)
		double h;	// Object height (for collision detection)
		vector<pair<double, double>> shape; // Object visual shape (List of points for polygon)
		Wired();
		virtual ~Wired();
		void render(bool global,bool highlight);
		int init(double x,double y,double w,double h,double r,vector<pair<double, double>> shape);
};

#endif

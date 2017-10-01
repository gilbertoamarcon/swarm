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
		Wired(double x,double y,double w,double h,double r,vector<pair<double, double>> shape);
		virtual ~Wired();
		void render(bool global,bool highlight, float r=1.0, float g=1.0, float b=1.0);
};

#endif

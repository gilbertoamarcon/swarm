#ifndef TEXTURED_H
#define TEXTURED_H
#include "includes.hpp"

/*
Textured object representation.
Textured objects are defined by a bitmap loaded from file (PNG, alpha channel allowed).
*/

class Textured{
	private:
		unsigned int texture[1];
		int loadGLTextures(char * filename);
	public:
		double x;	// Position coordinate x
		double y;	// Position coordinate y
		double t;	// Object heading theta (in Degrees)
		double w;
		double h;
		Textured();
		virtual ~Textured();
		void update(double x,double y,double w,double h,double r);
		void render(bool global,bool highlight);
		int init(double x,double y,double w,double h,double r,char * filename);
};

#endif

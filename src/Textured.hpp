#ifndef TEXTURED_H
#define TEXTURED_H
#include "includes.hpp"

class Textured{
	private:
		unsigned int texture[1];
		int loadGLTextures(char * filename);
	public:
		double x;
		double y;
		double w;
		double h;
		double t;
		Textured();
		virtual ~Textured();
		void update(double x,double y,double w,double h,double r);
		void render(bool global,bool highlight);
		int init(double x,double y,double w,double h,double r,char * filename);
};

#endif

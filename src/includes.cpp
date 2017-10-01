#include "includes.hpp"

// Angle Operations
void angle_wrap(double &input){
	if(abs(input) > 180)
		input -= 360*(2*(input>0)-1);
}
double deg_to_rad(double input){
	return PI*input/180.0;
}
double rad_to_deg(double input){
	return 180.0*input/PI;
}

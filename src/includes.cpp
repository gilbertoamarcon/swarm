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
double gen_rand_range(double begin_range, double end_range){
	return begin_range + ((double)rand()/RAND_MAX)*(end_range-begin_range);
}

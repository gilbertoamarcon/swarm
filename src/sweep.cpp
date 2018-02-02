#include "mlp.hpp"

#define FILE_BUFFER_SIZE 1048576

int main(int argc, char **argv){

	if(argc < 10){
		printf("ERROR: Missing imput arguments.\n");
		return 1;
	}

	// Memory buffer
	char buffer[FILE_BUFFER_SIZE];

	double *defs;

	// Parsing inputs
	int steps			= atoi(argv[1]);
	int ix				= atoi(argv[2]);
	int iy				= atoi(argv[3]);
	int out				= atoi(argv[4]);
	pair<double,double> rx(atof(argv[5]), atof(argv[6]));
	pair<double,double> ry(atof(argv[7]), atof(argv[8]));
	char *file_input	= argv[9];
	char *file_output	= argv[10];

	// Initializing MLP and loading weights
	Mlp mlp(file_input);

	// Allocating default input vector
	defs = new double[mlp.I];
	for(int i = 0; i < mlp.I; i++)
		defs[i] = atof(argv[11+i]);

	// Sweeping through the MLP inputs
	mlp.sweep(buffer, defs, steps, ix, iy, out, rx, ry);

	// Storing sweep results
	FILE *file 	= fopen(file_output,"w");
	if(file == NULL){
		printf("Error writing file '%s'.\n",file_output);
		return 1;
	}
	printf("Saving file '%s'...\n",file_output);	
	fprintf(file,"%s\n",buffer);	
	fclose(file);
	printf("Weights written.\n");

	return 0;

}
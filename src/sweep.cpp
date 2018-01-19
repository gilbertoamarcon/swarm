#include "mlp.hpp"

#define FILE_BUFFER_SIZE 1048576

int main(int argc, char **argv){

	if(argc < 9){
		printf("ERROR: Missing imput arguments.\n");
		return 1;
	}

	// Memory buffer
	char sweep_buffer[FILE_BUFFER_SIZE];

	double *defs;

	// Parsing inputs
	int steps	= atoi(argv[1]);
	int ix		= atoi(argv[2]);
	int iy		= atoi(argv[3]);
	pair<double,double> rx(atof(argv[4]), atof(argv[5]));
	pair<double,double> ry(atof(argv[6]), atof(argv[7]));
	char *file_input	= argv[8];
	char *file_output	= argv[9];

	// Initializing MLP and loading weights
	Mlp mlp(file_input);

	// Allocating default input vector
	defs = new double[mlp.I];
	for(int i = 0; i < mlp.I; i++)
		defs[i] = atof(argv[10+i]);

	// Sweeping through the MLP inputs
	mlp.sweep(sweep_buffer, defs, steps, ix, iy, rx, ry);

	// Storing sweep results
	FILE *file 	= fopen(file_output,"w");
	if(file == NULL){
		printf("Error writing file '%s'.\n",file_output);
		return 1;
	}
	printf("Saving file '%s'...\n",file_output);	
	fprintf(file,"%s\n",sweep_buffer);	
	fclose(file);
	printf("Weights written.\n");

	return 0;

}
#include "mlp.hpp"

Mlp::Mlp(){}

void Mlp::init(int I,int J,int K,double iniRange){

	this->I = I;
	this->J = J;
	this->K = K;
	this->iniRange = iniRange;
	this->error = 0.0;

	// Weights
	weights = new Weights(I,J,K);

	// Internal variables
	u0 = new double[J];
	u1 = new double[K];
	y = new double[J+1];

	// Inputs
	x = new double[I+1];

	// Outputs
	o = new double[K];

}

void Mlp::randomize(){
	
	normal_distribution<double> distV(0,iniRange);
	normal_distribution<double> distW(0,iniRange/(J+1));

	// Initializing weights V
	for(int i = 0; i < J*(I+1); i++)
		weights->V[i] = distV(generator);

	// Initializing weights W
	for(int i = 0; i < K*(J+1); i++)
		weights->W[i] = distW(generator);
}

void Mlp::mutate(double range){
	
	normal_distribution<double> distV(0,range);
	normal_distribution<double> distW(0,range/(J+1));

	// Mutating weights V
	for(int i = 0; i < J*(I+1); i++)
		weights->V[i] += distV(generator);

	// Mutating weights W
	for(int i = 0; i < K*(J+1); i++)
		weights->W[i] += distW(generator);
}

void Mlp::eval(){

	x[I] = 1.0;

	// Computing HL output 
	for(int i = 0; i < J; i++){
		u0[i] = 0;
		for(int j = 0; j < I+1; j++)
			u0[i] += weights->V[(I+1)*i+j]*x[j];
		y[i] = atan(u0[i]);
	}

	// Computing OL output 
	y[J] = 1;
	for(int i = 0; i < K; i++){
		u1[i] = 0;
		for(int j = 0; j < J+1; j++)
			u1[i] += weights->W[(J+1)*i+j]*y[j];
		o[i] = atan(u1[i]);
	}

}

int Mlp::store(char *mlp_weights){

	FILE *file;	
	char fileBuffer[BUFFER_SIZE]; 

	file 	= fopen(mlp_weights,"w");
	if(file == NULL){
		printf("Error writing file '%s'.\n",mlp_weights);
		return 1;
	}

	printf("Saving file '%s'.\n",mlp_weights);
	
	fprintf(file,"mlp_weights\n");
	fprintf(file,"I:%d\n",I);
	fprintf(file,"J:%d\n",J);
	fprintf(file,"K:%d\n",K);

	// Storing weights V
	fprintf(file,"V:\n");
	for(int i = 0; i < J*(I+1); i++)
		fprintf(file,"%32.32f\n",weights->V[i]);

	// Storing weights W
	fprintf(file,"W:\n");
	for(int i = 0; i < K*(J+1); i++)
		fprintf(file,"%32.32f\n",weights->W[i]);
	
	printf("Weights written.\n");
	fclose(file);
	return 0;
}

int Mlp::load(char *mlp_weights){

	int aux = 0;
	FILE *file;	
	char fileBuffer[BUFFER_SIZE]; 

	file 	= fopen(mlp_weights,"r");
	if(file == NULL)
		return 1;

	// File Header
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Wrong file type.\n");
		return 1;
	}

	// I
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	aux = 0;
	while(fileBuffer[aux++] != ':');
	I = atoi(fileBuffer+aux);

	// J
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	aux = 0;
	while(fileBuffer[aux++] != ':');
	J = atoi(fileBuffer+aux);

	// K
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	aux = 0;
	while(fileBuffer[aux++] != ':');
	K = atoi(fileBuffer+aux);

	init(I,J,K,1.0);

	// Loading weights V
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	for(int i = 0; i < J*(I+1); i++){
		if(fgets(fileBuffer,BUFFER_SIZE,file) == NULL) 
			return(1);
		weights->V[i] = atof(fileBuffer);
	}

	// Loading weights W
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	for(int i = 0; i < K*(J+1); i++){
		if(fgets(fileBuffer,BUFFER_SIZE,file) == NULL) 
			return(1);
		weights->W[i] = atof(fileBuffer);
	}
	
	fclose(file);
	return 0;
}

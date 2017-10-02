#include "mlp.hpp"

Mlp::Mlp(){}

void Mlp::init(int I,int J,int K,int N,int C,double D,double iniRange){

	this->I = I;
	this->J = J;
	this->K = K;
	this->N = N;
	this->C = C;
	this->D = D;
	this->iniRange = iniRange;

	// Weights
	mainW = new Weights(I,J,K);
	bkpW = new Weights(I,J,K);

	// Internal variables
	u = new double[J];
	y = new double[J+1];
	delta_o = new double[K];
	delta_h = new double[J];

	// Inputs
	x = new double[I+1];

	// Outputs
	o = new double[K];

	srand(clock());

}

void Mlp::randomize(){
	
	normal_distribution<double> distV(0,iniRange);
	normal_distribution<double> distW(0,1.0/(J+1));

	// Initializing weights V
	for(int i = 0; i < J*(I+1); i++)
		mainW->V[i] = distV(generator);

	// Initializing weights W
	for(int i = 0; i < K*(J+1); i++)
		mainW->W[i] = distW(generator);
}

void Mlp::eval(){

	x[I] = 1.0;

	// Computing HL output 
	for(int i = 0; i < J; i++){
		u[i] = 0;
		for(int j = 0; j < I+1; j++)
			u[i] += mainW->V[(I+1)*i+j]*x[j];
		y[i] = 1.0/(1+exp(-u[i]));
	}

	// Computing OL output 
	y[J] = 1;
	for(int i = 0; i < K; i++){
		o[i] = 0;
		for(int j = 0; j < J+1; j++)
			o[i] += mainW->W[(J+1)*i+j]*y[j];
	}

}

void Mlp::train(double *s, double *d, int P){	

	// Random Start
	randomize();
	evalError(s,d,P);

	if(C != 0){
		descend(s,d,P,N/C);
		backupWeights(bkpW);

		for(int c = 1; c <= C; c++){
			printf("Candidate %9d/%9d ",C,C);
			randomize();
			evalError(s,d,P);
			printf("E %9.3f",mainW->E);
			descend(s,d,P,N/C);
			if(mainW->E == mainW->E && mainW->E < bkpW->E)
				backupWeights(bkpW);
			else	
				restoreBackupWeights(bkpW);
			printf("\n");
		}
	}

	descend(s,d,P,N);
}

void Mlp::descend(double *s, double *d, int P,int N){
	for(int n = 0; n < N; n++){
		itTrain(s,d,P,D);
		if(n%100 == 0) printf("Iteration %9d/%d (%05.2f %) E: %10.9f\n",n,N,100*(double)n/N,mainW->E);
	}
}

void Mlp::itTrain(double *s, double *d, int P,double stepSize){
	mainW->E = 0;
	for(int p = 0; p < P; p++){

		// Loading inputs
		for(int i = 0; i < I; i++)
			x[i] = s[p*I+i];

		// Feedforward
		eval();

		// OL delta error
		for(int k = 0; k < K; k++){
			delta_o[k] = d[p*K+k] - o[k];
			mainW->E += pow(delta_o[k],2);
		}

		// HL delta error
		for(int j = 0; j < J; j++){
			delta_h[j] = 0;
			for(int k = 0; k < K; k++)
				delta_h[j] += delta_o[k]*mainW->W[K*k+j];
			delta_h[j] *= (1-y[j])*y[j];
		}

		// Weights W iteration
		for(int k = 0; k < K; k++)
			for(int j = 0; j < J+1; j++)
				mainW->W[(J+1)*k+j] += stepSize*delta_o[k]*y[j];

		// Weights V iteration
		for(int j = 0; j < J; j++)
			for(int i = 0; i < I+1; i++)
				mainW->V[(I+1)*j+i] += stepSize*delta_h[j]*x[i];

	}
	mainW->E /= K*P;
}

void Mlp::backupWeights(Weights *bkpWeights){
	bkpWeights->E = mainW->E;
	for(int i = 0; i < J*(I+1); i++)
		bkpWeights->V[i] = mainW->V[i];
	for(int i = 0; i < K*(J+1); i++)
		bkpWeights->W[i] = mainW->W[i];
}

void Mlp::restoreBackupWeights(Weights *bkpWeights){
	mainW->E = bkpWeights->E;
	for(int i = 0; i < J*(I+1); i++)
		mainW->V[i] = bkpWeights->V[i];
	for(int i = 0; i < K*(J+1); i++)
		mainW->W[i] = bkpWeights->W[i];
}

void Mlp::evalError(double *s, double *d, int P){
	mainW->E = 0;
	for(int p = 0; p < P; p++){

		// Loading input
		for(int i = 0; i < I; i++)
			x[i] = s[p*I+i];

		// Feedforward
		eval();

		// OL Error
		for(int k = 0; k < K; k++)
			mainW->E += pow(d[p*K+k] - o[k],2);
	}
	mainW->E /= K*P;
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
	fprintf(file,"N:%d\n",N);
	fprintf(file,"C:%d\n",C);
	fprintf(file,"D:%f\n",D);

	// Storing weights V
	fprintf(file,"V:\n");
	for(int i = 0; i < J*(I+1); i++)
		fprintf(file,"%32.32f\n",mainW->V[i]);

	// Storing weights W
	fprintf(file,"W:\n");
	for(int i = 0; i < K*(J+1); i++)
		fprintf(file,"%32.32f\n",mainW->W[i]);
	
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

	// N
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	aux = 0;
	while(fileBuffer[aux++] != ':');
	N = atoi(fileBuffer+aux);

	// C
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	aux = 0;
	while(fileBuffer[aux++] != ':');
	C = atoi(fileBuffer+aux);

	// D
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	aux = 0;
	while(fileBuffer[aux++] != ':');
	D = atof(fileBuffer+aux);

	init(I,J,K,N,C,D,1.0);

	// Loading weights V
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	for(int i = 0; i < J*(I+1); i++){
		if(fgets(fileBuffer,BUFFER_SIZE,file) == NULL) 
			return(1);
		mainW->V[i] = atof(fileBuffer);
	}

	// Loading weights W
	if(fgets(fileBuffer, BUFFER_SIZE, file) == NULL){
		printf("Error: Error while reading file.\n");
		return 1;
	}
	for(int i = 0; i < K*(J+1); i++){
		if(fgets(fileBuffer,BUFFER_SIZE,file) == NULL) 
			return(1);
		mainW->W[i] = atof(fileBuffer);
	}
	
	fclose(file);
	return 0;
}

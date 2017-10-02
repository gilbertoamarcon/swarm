#ifndef __MLP_HPP__
#define __MLP_HPP__
#include "includes.hpp"

struct Weights{
		
	double E;

	// Weights
	double *V;
	double *W;

	Weights(int I,int J,int K){
		E = 1e99;
		V = new double[J*(I+1)];
		W = new double[K*(J+1)];
	}

	~Weights(){
		free(V);
		free(W);
	}

};

class Mlp{

	private:
		int I;		// Number of inputs
		int J;		// Number of HL units
		int K;		// Number of outputs
		int N;		// Number of epochs
		int C;		// Number of initial point candidates
		double D;	// Step size
		double iniRange;

		// Weights
		Weights *mainW;	// Main weights
		Weights *bkpW;	// Backup weights

		// Internal variables
		double *u;
		double *y;
		double *delta_o;
		double *delta_h;

		default_random_engine generator;

		// Eval current weight error
		void evalError(double *s, double *d, int P);

		// Gradient Step
		void itTrain(double *s, double *d, int P,double stepSize);

		// Backup Weights
		void backupWeights(Weights *bkpWeights);

		// Restore Backup Weights
		void restoreBackupWeights(Weights *bkpWeights);

		// Gradient descent
		void descend(double *s, double *d, int P,int N);

	public:

		// Inputs
		double *x;

		// Outputs
		double *o;

		// Constructor
		Mlp();

		void init(int I,int J,int K,int N,int C,double D,double iniRange);

		// Weight initialization
		void randomize();

		// Feedforward
		void eval();

		// Training process
		void train(double *s,double *d, int P);

		// Storing weights
		int store(char *mlp_weights);

		// Loading weights
		int load(char *mlp_weights);
};


#endif

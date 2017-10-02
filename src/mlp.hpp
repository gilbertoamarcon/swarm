#ifndef __MLP_HPP__
#define __MLP_HPP__
#include "includes.hpp"

struct Weights{

	// Weights
	double *V;
	double *W;

	Weights(int I,int J,int K){
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
		double iniRange;

		// Weights
		Weights *weights;	// weights

		// Internal variables
		double *u0;
		double *u1;
		double *y;

		default_random_engine generator;

	public:

		double error;

		// Inputs
		double *x;

		// Outputs
		double *o;

		// Constructor
		Mlp();

		void init(int I,int J,int K,double iniRange);

		// Weight initialization
		void randomize();

		// Weight mutation
		void mutate(double range);

		// Feedforward
		void eval();

		// Storing weights
		int store(char *mlp_weights);

		// Loading weights
		int load(char *mlp_weights);

		bool operator < (const Mlp& mlp) const{
			return (error < mlp.error);
		}
};


#endif

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
		int J;		// Number of HL units
		double iniRange;

		// Internal variables
		double *u0;
		double *u1;
		double *y;

	public:

		int I;		// Number of inputs
		int K;		// Number of outputs

		// Weights
		Weights *weights;	// weights

		double error;

		// Inputs
		double *x;

		// Outputs
		double *o;

		// Constructors
		Mlp(char *mlp_weights);
		Mlp(int I,int J,int K,double iniRange);
		Mlp(Mlp* mlp);

		void init(int I,int J,int K,double iniRange);

		void print_weights(char* str);

		void sweep(char* buffer, double *defs, int steps, int vx, int vy, int out, pair<double,double> lx, pair<double,double> ly);

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

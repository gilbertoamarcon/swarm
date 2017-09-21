#ifndef MAP_H
#define MAP_H
#include "includes.hpp"

class Node{

	public:
		int index;
		float cost;
		float heuristic;
		float est_cost;
		Node *parent;

		Node(int index,float cost,float heuristic,Node *parent){
			this->index = index;
			this->cost = cost;
			this->heuristic = heuristic;
			this->est_cost = cost+heuristic;
			this->parent = parent;
		};

		virtual ~Node(){};
};

class Map{

	private:

		// Path finding variables
		#if PATH_DEBUG
			float *pathCost;
			bool *visited;
		#endif
		list<Node*> frontier;
		list<Node*> closed;

		// Textures 
		unsigned int texture[5];
		int loadGLTexture(char * filename,int layer);

		// Path finding 
		void pos2index(double x,double y,int *i,int *j);
		void index2pos(int i,int j,double *x,double *y);
		int coordinate2index(int i,int j);
		void index2coordinate(int index,int *i,int *j);
		int checkIndex(int i,int j);
		int checkCorner(int iC,int i,int jC,int j);
		float distanceHeuristic(int i,int j,int iF,int jF);
		void expand(Node *parent,int iC,int jC,int iF,int jF,int i,int j,float cost);
		int search(int iI,int jI,int iF,int jF,vector<int> *iP,vector<int> *jP);

	public:
		int cols;
		int rows;
		int *map;
		double w;
		double h;
		Map();
		virtual ~Map();
		int nearestEmptyPos(double *x,double *y);
		int checkCol(double x,double y,double w,double h);
		void getHQ(double &x,double &y);
		void render();
		int loadMap(char *filename);
		int init(double w,double h,char * filename);

		// Path finding 
		int path(double xI,double yI,double xF,double yF,vector<double> *xP,vector<double> *yP);
};

#endif

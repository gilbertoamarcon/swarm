#ifndef INCLUDES_H
#define INCLUDES_H
#include "GL/freeglut.h"
#include "GL/gl.h"
#include <X11/Xlib.h>
#include <png.h>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <list>
#include <vector>
#include <set>
#include <map>
#include <tuple>
#include <sstream>
#include <algorithm>
#include <cstdio>
#include <random>
#include <chrono>

using namespace std;

// MLP Parameters
#define MLP_I			1 // Number of inputs
#define MLP_J			1 // Number of HL units
#define MLP_K			1 // Number of outputs
#define MLP_INIT_RANGES	1.0

// Simulation Parameters
#define EPOCH_STEPS		300

// Evolution Parameters
#define POP_SIZE		15
#define NUM_PARENTS		5
#define NUM_EPOCHS		50000
#define MUTATION_RANGE	0.01

// Sim Parameters
#define NUM_LEADERS		1
#define NUM_ROBOTS		1
#define ROBOT_SPAWN_RNG	100
#define GOAL_SPAWN_RNG	300
#define ROBOT_VEL		1.00
#define ROBOT_STEERING 	0.05
#define DEFAULT_TOL		0.05
#define PI				3.14159265
#define WORLD_SIZE_X	200
#define WORLD_SIZE_Y	200

// Flock Parameters
#define REP_RADIUS 		20
#define ORI_RADIUS 		50
#define ATR_RADIUS 		100

// File Parameters
#define BUFFER_SIZE		256

// Video Parameters
#define WINDOW_TITLE	"Video"
#define FULL_SCREEN		0
#define SIM_STEP_TIME	1.000
// #define SIM_STEP_TIME	0.001

// Camera Parameters
#define CAM_SPEED		10
#define SCN_SCALE		0.50

// Keyboard Commands
#define EXIT			27
#define KEY_UP			'w'
#define KEY_DOWN		's'
#define KEY_LEFT		'a'
#define KEY_RIGHT		'd'
#define KEY_ZOON_IN		'e'
#define KEY_ZOON_OUT	'q'
#define RESET_ZOOM		'z'
#define RESET_VIEW_POS	'h'
#define LOCK_VIEW_ROBOT	'v'

// Angle Operations
void angle_wrap(double &input);
double deg_to_rad(double input);
double rad_to_deg(double input);

// Random number generation
double gen_rand_range(double begin_range, double end_range);

#endif

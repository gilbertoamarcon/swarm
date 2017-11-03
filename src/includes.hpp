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
#include <fstream>
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

// Learning Switch
#define LEARNING 		1

// Data Collection
#define COLLECT_DATA 	0
#define NUM_SAMPLES 	20 // Number of epochs data is collected (-1 for infinity)
#define AUTOSAVE 		1 // For training, save weights to file when num_epocs reached
#define AUTOLOAD		0 // For evaluation of a trained network, start with loaded weights

// I/O Parameters
#define WEIGHTS_FILE	"data/MLP_weights_BASE_24R_16L_M.txt"
#define DATA_FILE		"data/MLP_BASE_12R_8L_M.txt"

// MLP Parameters
#define MLP_I			6 // Number of inputs
#define MLP_J			7 // Number of HL units
#define MLP_K			1 // Number of outputs
#define MLP_INIT_RANGES	1.0

// Simulation Parameters
#define EPOCH_STEPS		700

// Reward Function			// Choose one
#define CLASSIC_REW		1 	// Distance to goal weighted by time
#define ALTERNATE_REW	0 	// Minimum Dist - Reaching the goal is rewarded, not staying there

// Evolution Parameters
#define POP_SIZE		15
#define NUM_PARENTS		5
#define NUM_EPOCHS		500
#define MUTATION_RANGE	0.0 // Set to zero when testing

// Sim Parameters
#define NUM_LEADERS		2//16
#define NUM_ROBOTS		8//40
#define ROBOT_SPAWN_RNG	100
#define GOAL_SPAWN_RNG	800
#define ROBOT_VEL		1.00
#define LEADER_VEL		1.20
#define ROBOT_STEERING 	0.05
#define DEFAULT_TOL		0.05
#define PI				3.14159265
#define WORLD_SIZE_X	500
#define WORLD_SIZE_Y	500
#define NUM_GOALS		1
#define NUM_OBSTACLES	10
#define COMM_MODEL 		'M'
#define N_TOP 			6

// Flock Parameters
#define REP_RADIUS 		400
#define ORI_RADIUS 		2500
#define ATR_RADIUS 		10000
#define VIS_ANGLE		(2 * PI / 3)
#define SWARM_PULL		0 // amount of influence agents have on leaders on [0, 1] scale

// File Parameters
#define BUFFER_SIZE		256

// Video Parameters
#define WINDOW_TITLE	"Video"
#define FULL_SCREEN		0
#define SIM_STEP_TIME	1.000

// Light trail for the lulz
#define ENABLE_TRAIL 	0
#define SWARM_TRAIL 	1
#define TRAIL_LENGTH	100.0

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
#define LOCK_VIEW_ROBOT	'l'
#define SAVE_WEIGHTS	'c'
#define LOAD_WEIGHTS	'v'

// Angle Operations
void angle_wrap(double &input);
double deg_to_rad(double input);
double rad_to_deg(double input);

// Random number generation
double gen_rand_range(double begin_range, double end_range);

#endif

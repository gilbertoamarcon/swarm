#ifndef INCLUDES_H
#define INCLUDES_H
#include "GL/freeglut.h"
#include "GL/gl.h"
#include <X11/Xlib.h>
#include <png.h>
#include <cstdio>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <list>
#include <vector>
#include <sstream>
#include <algorithm>
#include <tuple>
#include <cmath>

using namespace std;

// Sim Parameters
#define NUM_ROBOTS		5
#define SPAWN_RANGE		250
#define ROBOT_VEL		1.00
#define DEFAULT_TOL		0.05
#define PI				3.14159265

// File Parameters
#define BUFFER_SIZE		256

// Video Parameters
#define WINDOW_TITLE	"Video"
#define FULL_SCREEN		1
#define SIM_STEP_TIME	10

// Camera Parameters
#define CAM_SPEED		10
#define SCN_SCALE		0.2

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

#endif

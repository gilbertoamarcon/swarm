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

using namespace std;

// Sim Parameters
#define MAX_ROBOTS		50
#define TIMER			30
#define SPAWN_RANGE		30
#define IRON_COST		1500
#define POWER_COST		1000
#define ROBOT_VEL		0.25
#define DEFAULT_TOL		0.05
#define LIFE_DEC_COL	0.001
#define LIFE_GAIN_HQ	0.005

// Search parameters
#define INFLATION		1

// File Parameters
#define BUFFER_SIZE		256

// Video Parameters
#define WINDOW_TITLE	"Video"
#define FULL_SCREEN		1
#define FRAME_TIME		10

// Camera Parameters
#define SCN_SCALE		0.2
#define PATH_DEBUG		1

// Keyboard Commands
#define EXIT			27
#define KEY_UP			'w'
#define KEY_DOWN		's'
#define KEY_LEFT		'a'
#define KEY_RIGHT		'd'
#define KEY_ZOON_IN		'e'
#define KEY_ZOON_OUT	'q'
#define BUILD_NEW		'b'
#define VIEW_HQ			'h'
#define VIEW_ROBOT		'v'
#define RESET_ZOOM		'z'

#endif

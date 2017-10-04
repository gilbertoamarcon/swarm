#include "includes.hpp"
#include "Wired.hpp"
#include "Textured.hpp"
#include "Robot.hpp"

Textured cursor;
Textured flag;
vector<Robot> flock;
vector<Mlp> mlps;

// Steps
int num_steps			= 0;
int current_epoch		= 0;
int current_mlp			= 0;

// Status
double origin_x			= 0;
double origin_y			= 0;

double old_mouse_x		= 0;
double old_mouse_y		= 0;
double mouse_gnd_x		= 0;
double mouse_gnd_y		= 0;

// Screen Resolution
int window_w			= 1000;
int window_h			= 1000;

// Camera position/motion
double view_x			= 0;
double view_y			= 0;
double x_min			= 0;
double x_max			= 0;
double y_min			= 0;
double y_max			= 0;
double cam_speed		= CAM_SPEED;
double scn_scale		= SCN_SCALE;

// User input flags
bool mouse_l			= 0;
bool mouse_m			= 0;
bool mouse_r			= 0;
bool lock_view_robot	= 0;

// Keyboard mov flags
bool keyLeft			= 0;
bool keyRight			= 0;
bool keyDown			= 0;
bool keyUp				= 0;
bool keyZoomIn			= 0;
bool keyZoomOut			= 0;
bool saveWeights		= 0;
bool loadWeights		= 0;

// Mouse mov flags
bool mouseLeft			= 0;
bool mouseRight			= 0;
bool mouseDown			= 0;
bool mouseUp			= 0;

bool selecting			= false;
double selX1			= 0;
double selY1			= 0;
double selX2			= 0;
double selY2			= 0;

// I/O
ofstream datafile;
char statusBuffer[BUFFER_SIZE];

void spawn_world();

double compute_error();

void getScreenResolution(int &h, int &v);

void iniGl();

void areaSelect();

void singleSelect();

void mouseButton(int b,int s,int x,int y);

void cursorUpdate(int x,int y);

void mouseMove(int x, int y);

void mouseAction(int x, int y);

void set_goal(int x, int y);

void glutMouseFunc(int button, int state, int x, int y);

void keyPressed(unsigned char key, int x, int y);

void keyReleased(unsigned char key, int x, int y);

void updateValues(int n);

void RenderScene();

void codeTestOvunc();

int main(int argc, char **argv){

	srand(clock());

	// Initializing graphics
	getScreenResolution(window_w,window_h);
	glutInit(&argc, argv);
	iniGl();

	// Agent Shape
	vector<pair<double, double>> shape;
	shape.push_back(pair<double, double>(0.0,0.50));
	shape.push_back(pair<double, double>(1.0,0.75));
	shape.push_back(pair<double, double>(1.0,0.25));

	view_x = origin_x;
	view_y = origin_y;

	// Cursor and goal objects
	if(cursor.init(0,0,64,64,0,"img/cursor.png")) return 0;
	if(flag.init(0,0,16,16,0,"img/flag.png")) return 0;

	// Initializing mlps
	for(int i = 0; i < POP_SIZE; i++){
		Mlp mlp(MLP_I,MLP_J,MLP_K,MLP_INIT_RANGES);
		mlp.randomize();
		mlps.push_back(mlp);
	}

	// Initializing robots
	for(int i = 0; i < NUM_ROBOTS; i++){
		bool leader = false;
		if(i < NUM_LEADERS)
			leader = true;
		Robot robot(0.0,0.0,2,2,0,ROBOT_VEL,ROBOT_STEERING,shape,&flock, REP_RADIUS, ORI_RADIUS, ATR_RADIUS, leader);
		flock.push_back(robot);
	}

	// Open data file
  	datafile.open(DATA_FILE);

	spawn_world();

	// Main loop
	glutMainLoop();

	return 0;
}

void spawn_world(){

	// Positioning robots
	for(auto const &r : flock){
		double rx = origin_x + ((double)rand()/RAND_MAX-0.5)*ROBOT_SPAWN_RNG;
		double ry = origin_y + ((double)rand()/RAND_MAX-0.5)*ROBOT_SPAWN_RNG;
		double rt = ((double)rand()/RAND_MAX-0.5)*360.0;
		r.respawn(rx, ry, rt, &(mlps.at(current_mlp)));
	}

	// New Goal Rally Point
	double gx = origin_x;
	double gy = origin_y;
	int spawn_location = rand()%4;
	if(spawn_location == 0){
		gx += gen_rand_range(ROBOT_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
		gy += gen_rand_range(-GOAL_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
	}
	if(spawn_location == 1){
		gx -= gen_rand_range(ROBOT_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
		gy += gen_rand_range(-GOAL_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
	}
	if(spawn_location == 2){
		gx += gen_rand_range(-GOAL_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
		gy += gen_rand_range(ROBOT_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
	}
	if(spawn_location == 3){
		gx += gen_rand_range(-GOAL_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
		gy -= gen_rand_range(ROBOT_SPAWN_RNG/2,GOAL_SPAWN_RNG/2);
	}
	set_goal(gx, gy);

}

double compute_error(){
	double error = 0.0;
	for(auto const &r : flock)
		if(!(r.leader))
			error += r.acc_dist;
	error /= (NUM_ROBOTS-NUM_LEADERS);
	error /= EPOCH_STEPS;
	return error;
}

// Get the horizontal and vertical screen sizes in pixel
void getScreenResolution(int &h, int &v){
	if(FULL_SCREEN){
		Display* d = XOpenDisplay(NULL);
		h   = DisplayWidth(d,0);
		v   = DisplayHeight(d,0);
	}
}

// OpenGL initialization
void iniGl(){
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(window_w,window_h);
	glutCreateWindow(WINDOW_TITLE);
	glutMouseFunc(&mouseButton);
	glutMotionFunc(&mouseAction);
	glutPassiveMotionFunc(&mouseMove);
	glutKeyboardFunc(&keyPressed);
	glutKeyboardUpFunc(&keyReleased);
	glutDisplayFunc(&RenderScene);
	glutIdleFunc(&RenderScene);
	glutTimerFunc(1,updateValues,0);
	glMatrixMode(GL_PROJECTION);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glDisable(GL_ALPHA_TEST);
	gluOrtho2D(x_min,x_max,y_min,y_max);
	if(FULL_SCREEN)
		glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);
}

// Mouse click selection
void singleSelect(){
	for(auto &r : flock){
		if(r.x-r.w/2 > selX1){r.selected = false;continue;}
		if(r.x+r.w/2 < selX1){r.selected = false;continue;}
		if(r.y-r.h/2 > selY1){r.selected = false;continue;}
		if(r.y+r.h/2 < selY1){r.selected = false;continue;}
		r.selected = true;
	}
}

// Mouse area selection
void areaSelect(){
	double minX = (selX2 > selX1)?selX1:selX2;
	double maxX = (selX2 > selX1)?selX2:selX1;
	double minY = (selY2 > selY1)?selY1:selY2;
	double maxY = (selY2 > selY1)?selY2:selY1;
	for(auto &r : flock){
		if(r.x-r.w/2 < minX) continue;
		if(r.x+r.w/2 > maxX) continue;
		if(r.y-r.h/2 < minY) continue;
		if(r.y+r.h/2 > maxY) continue;
		r.selected = true;
	}
}

// Screen to space conversion
double scn2space(int a){
	return scn_scale*((double)a);
}

// Updating cursor position variables
void cursorUpdate(int x,int y){	
	cursor.x	= x;
	cursor.y	= y;
	mouseLeft	= (x == 0);
	mouseRight	= (x == window_w-1);
	mouseUp		= (y == 0);
	mouseDown	= (y == window_h-1);
	mouse_gnd_x = view_x-scn2space(x-window_w/2);
	mouse_gnd_y = view_y+scn2space(y-window_h/2);
}

// When mouse moves
void mouseMove(int x,int y){
	cursorUpdate(x,y);
	old_mouse_x	= x;
	old_mouse_y	= y;
}

// Mouse click trigger actions
void mouseButton(int b,int s,int x,int y){
	switch (b){
		case GLUT_LEFT_BUTTON:
			if(s == GLUT_DOWN){
				selX1 = mouse_gnd_x;
				selY1 = mouse_gnd_y;
				singleSelect();
				mouse_l = 1;
			}
			else{
				selX2 = mouse_gnd_x;
				selY2 = mouse_gnd_y;
				areaSelect();
				mouse_l = 0;
			}
			break;
		case GLUT_MIDDLE_BUTTON:
			if(s == GLUT_DOWN)
				mouse_m = 1;
			else
				mouse_m = 0;
			break;
		case GLUT_RIGHT_BUTTON:
			if(s == GLUT_DOWN){
				mouse_r = 1;
				set_goal(mouse_gnd_x, mouse_gnd_y);
			}
			else
				mouse_r = 0;
			break;
		default:
			break;
	}
}

// Sustained mouse click actions
void mouseAction(int x,int y){

	cursorUpdate(x,y);

	double delta_mouse_x = x-old_mouse_x;
	double delta_mouse_y = y-old_mouse_y;

	if(mouse_m){
		view_x += -5*scn2space(delta_mouse_x);
		view_y +=  5*scn2space(delta_mouse_y);
	}
	if(mouse_l){
	}
	if(mouse_r){
	}
	old_mouse_x = x;
	old_mouse_y = y;
}

// Set rally goal position
void set_goal(int x, int y){
	for(auto &r : flock)
		if(r.selected)
			r.set_goal_target_pos(x,y);
	flag.x = x;
	flag.y = y;
}

// When keyboard pressed
void keyPressed(unsigned char key, int x, int y){
	switch (key){
		case EXIT:
			exit(0);
			break;
		case KEY_LEFT:
			keyLeft = 1;
			break;
		case KEY_RIGHT:
			keyRight = 1;
			break;
		case KEY_DOWN:
			keyDown = 1;
			break;
		case KEY_UP:
			keyUp = 1;
			break;
		case KEY_ZOON_IN:
			keyZoomIn = 1;
			break;
		case KEY_ZOON_OUT:
			keyZoomOut = 1;
			break;
		case RESET_ZOOM:
			scn_scale = SCN_SCALE;
			break;
		case RESET_VIEW_POS:
			view_x = origin_x;
			view_y = origin_y;
			break;
		case LOCK_VIEW_ROBOT:
			lock_view_robot = !lock_view_robot;
			break;
		case SAVE_WEIGHTS:
			saveWeights = 1;
			break;
		case LOAD_WEIGHTS:
			loadWeights = 1;
			break;
		default:
			break;
	}
}

// When keyboard released
void keyReleased(unsigned char key, int x, int y){
	switch (key){
		case KEY_LEFT:
			keyLeft = 0;
			break;
		case KEY_RIGHT:
			keyRight = 0;
			break;
		case KEY_DOWN:
			keyDown = 0;
			break;
		case KEY_UP:
			keyUp = 0;
			break;
		case KEY_ZOON_IN:
			keyZoomIn = 0;
			break;
		case KEY_ZOON_OUT:
			keyZoomOut = 0;
			break;
		default:
			break;
	}
}

void updateValues(int n){

	// Frame limiter
	if(current_epoch < NUM_EPOCHS)
		glutTimerFunc(0.001,updateValues,0);
	else
		glutTimerFunc(SIM_STEP_TIME,updateValues,0);

	num_steps++;
	if(num_steps == EPOCH_STEPS){
		num_steps = 0;

		// Error evaluation
		mlps.at(current_mlp).error = compute_error();

		// Write error to data file
		#if COLLECT_DATA
			datafile << mlps.at(current_mlp).error << ",";
		#endif

		// Swapping MLP
		current_mlp++;

		// All MLPs evaluated, mutation and selection
		if(current_mlp == POP_SIZE){
			current_mlp = 0;

			sort(mlps.begin(),mlps.end());

			char str[BUFFER_SIZE];
			double sum = 0;
			for(int i = 0; i < NUM_PARENTS; i++){
  				strcpy (str,"");
				// mlps.at(i).print_weights(str);
				sum += mlps.at(i).error;
				printf("R%d[%08.3f]: %s",i,mlps.at(i).error,str);
			}
			printf("\nAVG: %f", sum/NUM_PARENTS);
			printf("\n");

			// Erasing the worst mlps
			mlps.erase(mlps.begin()+NUM_PARENTS,mlps.end());

			// Reproducing the best mlps
			for(int i = 0; i < POP_SIZE-NUM_PARENTS; i++){

				// Parent selection
				Mlp mlp(&mlps.at(rand()%NUM_PARENTS));

				// Mutation
				mlp.mutate(MUTATION_RANGE);

				// Inserting into population
				mlps.insert(mlps.end(), mlp);

			}
			#if COLLECT_DATA
				datafile << endl;
			#endif
			current_epoch++;
		}

		spawn_world();

	}

	// Camera view will track specific flock
	if(lock_view_robot){
		for(auto const &r : flock)
			if(r.selected){
				view_x = r.x;
				view_y = r.y;
				break;
			}
	}

	// Moving the camera given the camera speed
	if(mouseLeft	|	keyLeft)	view_x += cam_speed*scn_scale;
	if(mouseRight	|	keyRight)	view_x -= cam_speed*scn_scale;
	if(mouseDown	|	keyDown)	view_y += cam_speed*scn_scale;
	if(mouseUp		|	keyUp)		view_y -= cam_speed*scn_scale;

	// Zoom
	if(keyZoomIn)
		scn_scale /= 1.05;
	if(keyZoomOut)
		scn_scale *= 1.05;

	// Save NN weights
	if(saveWeights){
		mlps.at(current_mlp).store(WEIGHTS_FILE);
		saveWeights = 0;
	}

	if(loadWeights){
		for(auto &mlp: mlps)
			mlp.load(WEIGHTS_FILE);
		loadWeights = 0;
	}

	// Updating camera projection parameters
	x_min = -view_x - scn_scale*window_w/2;
	x_max = -view_x + scn_scale*window_w/2;
	y_min = -view_y - scn_scale*window_h/2;
	y_max = -view_y + scn_scale*window_h/2;

	// Updating flock status
	double weight = 1-cos(PI*((double)num_steps/EPOCH_STEPS));
	for(auto &r : flock)
		r.update(weight);

	// String printed in the screen corner
	sprintf(statusBuffer,"Number of robots: %02d Epoch: %06d/%06d MLP: %02d/%02d Steps: %03d/%03d Weight: %4.2f",flock.size(),current_epoch,NUM_EPOCHS,current_mlp,POP_SIZE,num_steps,EPOCH_STEPS,weight);

}

// Scene rendering
void RenderScene(){

	// Clearing screen
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Space coordinates
	glPushMatrix();

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(x_min,x_max,y_min,y_max);
		glMatrixMode(GL_MODELVIEW);

		// World bounds
		glBegin(GL_LINE_LOOP);
			glVertex2f(-WORLD_SIZE_X,-WORLD_SIZE_Y);
			glVertex2f( WORLD_SIZE_X,-WORLD_SIZE_Y);
			glVertex2f( WORLD_SIZE_X, WORLD_SIZE_Y);
			glVertex2f(-WORLD_SIZE_X, WORLD_SIZE_Y);
		glEnd();

		// Light Trail
		#if ENABLE_TRAIL
		for(auto const &r : flock){
			for (int i=0; i < r.prevCoords.size()-1; i++){
				if(r.leader || SWARM_TRAIL){
					glLineWidth(2.5*i/TRAIL_LENGTH); 
					glBegin(GL_LINES);
					glColor3f(i/TRAIL_LENGTH, !r.leader*SWARM_TRAIL*i/TRAIL_LENGTH, !r.leader*SWARM_TRAIL*i/TRAIL_LENGTH);
					glVertex3f(-r.prevCoords[i].first, -r.prevCoords[i].second, 0);
					glVertex3f(-r.prevCoords[i+1].first, -r.prevCoords[i+1].second, 0);
					glEnd();
					}
				}
			}
		#endif 

		// Drawing robots
		for(auto const &r : flock)
			r.render_robot();

		// Goal flag
		flag.render(1,0);

		// Mouse selection
		if(mouse_l){
			glLineWidth(2);
			glColor4f(0,0,1,0.75);
			glBegin(GL_LINE_LOOP);
				glVertex2f(-selX1,-selY1);
				glVertex2f(-mouse_gnd_x,-selY1);
				glVertex2f(-mouse_gnd_x,-mouse_gnd_y);
				glVertex2f(-selX1,-mouse_gnd_y);
			glEnd();
			glColor4f(0,0,1,0.25);
			glBegin(GL_QUADS);
				glVertex2f(-selX1,-selY1);
				glVertex2f(-mouse_gnd_x,-selY1);
				glVertex2f(-mouse_gnd_x,-mouse_gnd_y);
				glVertex2f(-selX1,-mouse_gnd_y);
			glEnd();
			glLineWidth(1);
		}

	glPopMatrix();

	// Screen coordinates
	glPushMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0,window_w,window_h,0);
		glMatrixMode(GL_MODELVIEW);
		glTranslatef(0,0,0);
		cursor.render(0,0);
	glPopMatrix();

	// Status display
	glColor4f(0,0,0,0.75);
	glPushMatrix();
		glScalef(350,24,1);
		glBegin(GL_QUADS);
			glVertex2f(0,0);
			glVertex2f(1,0);
			glVertex2f(1,1);
			glVertex2f(0,1);
		glEnd();
	glPopMatrix();

	glColor3f(1,1,1);
	glRasterPos2i(3,15);
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glutBitmapString(GLUT_BITMAP_8_BY_13,statusBuffer);

	glutSwapBuffers();
}

#include "includes.hpp"
#include "Wired.hpp"
#include "Textured.hpp"
#include "Robot.hpp"

Textured cursor;
vector<Textured> flags;
vector<Textured> blocks;
vector<Robot> flock;
vector<Mlp> mlps;
vector<pair<int, int>> goals;
vector<pair<int, int>> obstacles;

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
int window_w			= 800;
int window_h			= 600;

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
bool keySpeedUp 		= 0;
bool keySlowDown 		= 0;

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

// Parameters as global variables
string weights_file = WEIGHTS_FILE;
string distance_file_name = DISTANCE_FILE;
string count_file_name = COUNT_FILE;
int num_leaders = NUM_LEADERS;
int num_robots = NUM_ROBOTS;
int num_epochs = NUM_EPOCHS;
char comm_model = COMM_MODEL;
double simSpeed = SIM_STEP_TIME;
// ------------------------------
bool autosave = AUTOSAVE;
bool autoload = AUTOLOAD;
double mutation_range = MUTATION_RANGE;

// I/O
ofstream distance_file_stream;
ofstream count_file_stream;
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

void set_goals(vector<pair<int, int>> &points);

void set_obstacles(vector<pair<int, int>> &points);

void glutMouseFunc(int button, int state, int x, int y);

void keyPressed(unsigned char key, int x, int y);

void keyReleased(unsigned char key, int x, int y);

void updateValues(int n);

void RenderScene();

void cl_arguments(int argc, char **argv);

int main(int argc, char **argv){
	srand(clock());
	// Get command line arguments
	cl_arguments(argc, argv);
	// Initializing graphics
	if (VISUALIZATION){
		getScreenResolution(window_w,window_h);
		glutInit(&argc, argv);
		iniGl();
	}


	// Agent Shape
	vector<pair<double, double>> shape;
	shape.push_back(pair<double, double>(0.0,0.50));
	shape.push_back(pair<double, double>(1.0,0.75));
	shape.push_back(pair<double, double>(1.0,0.25));

	view_x = origin_x;
	view_y = origin_y;

	// Cursor and goal objects
	if(cursor.init(0,0,64,64,0,"img/cursor.png")) return 0;
	for (int i = 0; i < NUM_GOALS; i++){
		Textured flag;
		if(flag.init(0,0,16,16,0,"img/flag.png")) return 0;
		flags.push_back(flag);
	}
	for (int i = 0; i < NUM_OBSTACLES; i++){
		Textured block;
		if(block.init(0,0,16,16,0,"img/block.png")) return 0;
		blocks.push_back(block);
	}

	// Initializing mlps
	for(int i = 0; i < POP_SIZE; i++){
		Mlp mlp(MLP_I,MLP_J,MLP_K,MLP_INIT_RANGES);
		mlp.randomize();
		mlps.push_back(mlp);
	}

	// Initializing robots
	for(int i = 0; i < num_robots; i++){
		bool leader = false;
		int group = i % NUM_GOALS;
		float velocity = ROBOT_VEL;

		if(i < num_leaders){
			leader = true;
			velocity = LEADER_VEL;
		}

		Robot robot(0.0,0.0,2,2,0,velocity,ROBOT_STEERING,shape,&flock, REP_RADIUS, ORI_RADIUS, ATR_RADIUS, leader, group);
		robot.comm_model = comm_model;
		flock.push_back(robot);
	}

	// Open data file
  	distance_file_stream.open(distance_file_name.c_str());
  	count_file_stream.open(count_file_name.c_str());

	spawn_world();

	// Main loop
	
	if (VISUALIZATION)
		glutMainLoop();
	else{
		while(current_epoch < num_epochs)
			updateValues(0);
	}

	return 0;
}

void cl_arguments(int argc, char **argv){
	if (argc < 2)
		return;

	if (strcmp(argv[1], "TRAIN")==0){
		autosave = 1;
		autoload = 0;
	}
	else if (strcmp(argv[1], "TEST")==0){
		mutation_range = 0;
		autosave = 0;
		autoload = 1;
	}

	if (argc >= 4) {
		num_leaders = atoi(argv[2]);
		num_robots = atoi(argv[3]);
	}
	if (argc >= 5) 
		num_epochs = atoi(argv[4]);
	if (argc >= 7) 
		comm_model = *argv[6];

	if (weights_file == ""){
		char buffer [100];
		sprintf(buffer, "data/Weights/%s%d_WEIGHTS_R%d_L%d_E%d.txt", EXP_FOLDER, atoi(argv[5]), num_robots, num_leaders, num_epochs);
		weights_file = buffer;
	}
	if (distance_file_name == ""){
		char buffer [100];
		if (strcmp(argv[1], "TRAIN") == 0)
			sprintf(buffer, "data/Performances/%s%d_TRAINING_DATA_%c_R%d_L%d_E%d.txt", EXP_FOLDER, atoi(argv[5]), comm_model, num_robots, num_leaders, num_epochs);
		else	
			sprintf(buffer, "data/Performances/%s%d_TEST_DATA_%c_R%d_L%d_E%d.txt", EXP_FOLDER, atoi(argv[5]), comm_model, num_robots, num_leaders, num_epochs);
		distance_file_name = buffer;
	}
	if (count_file_name == ""){
		char buffer [100];
		if (strcmp(argv[1], "TRAIN") == 0)
			sprintf(buffer, "data/Performances/%s%d_TRAINING_COUNT_DATA_%c_R%d_L%d_E%d.txt", EXP_FOLDER, atoi(argv[5]), comm_model, num_robots, num_leaders, num_epochs);
		else	
			sprintf(buffer, "data/Performances/%s%d_TEST_COUNT_DATA_%c_R%d_L%d_E%d.txt", EXP_FOLDER, atoi(argv[5]), comm_model, num_robots, num_leaders, num_epochs);
		count_file_name = buffer;
	}




}

void spawn_world(){

	// Positioning robots
	for(auto const &r : flock){
		double rx = origin_x + ((double)rand()/RAND_MAX-0.5)*ROBOT_SPAWN_RNG;
		double ry = origin_y + ((double)rand()/RAND_MAX-0.5)*ROBOT_SPAWN_RNG;
		double rt = 0;//((double)rand()/RAND_MAX-0.5)*360.0;
		r.respawn(rx, ry, rt, &(mlps.at(current_mlp)));
	}

	// New Goal Rally Point
	goals.clear();
	obstacles.clear();
	for (int i = 0; i < NUM_GOALS; i++){
		double th = rand() % 360 * PI/180;
		goals.push_back(pair<int, int>(sin(th) * GOAL_SPAWN_RNG, cos(th) * GOAL_SPAWN_RNG));
	}
	for (int i = 0; i < NUM_OBSTACLES; i++){
		double x = gen_rand_range(-WORLD_SIZE_X/2, WORLD_SIZE_X/2);
		double y = gen_rand_range(-WORLD_SIZE_Y/2, WORLD_SIZE_Y/2);
		obstacles.push_back(pair<int, int>(x, y));
	}
	set_goals(goals);
	set_obstacles(obstacles);

}

double compute_error(){
	double error = 0.0;
	for(auto const &r : flock)
		if(!(r.leader))
			error += r.acc_dist;
	error /= (num_robots-num_leaders);
	error /= EPOCH_STEPS;
	return error;
}

double count_robots_at_goal(){
	int count = 0;
	for(auto const &r : flock)
		if(!(r.leader) && r.is_within_goal_radius(flags)) 
			count++;

	return ((double)count) / (num_robots - num_leaders);
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
			if(s == GLUT_DOWN && NUM_GOALS == 1){
				mouse_r = 1;
				vector<pair<int, int>> points;
				points.push_back(pair<int, int>(mouse_gnd_x, mouse_gnd_y));
				set_goals(points);
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
void set_goals(vector<pair<int, int>> &points){
	for (int i = 0; i < NUM_GOALS; i++){
		flags[i].x = points[i].first;
		flags[i].y = points[i].second;
	}

	for(auto &r : flock)
		if (r.selected)
			r.set_goal_target_pos(flags[r.goal_group].x, flags[r.goal_group].y);
}

// Set obstacle positions
void set_obstacles(vector<pair<int, int>> &points){
	for (int i = 0; i < NUM_OBSTACLES; i++){
		blocks[i].x = points[i].first;
		blocks[i].y = points[i].second;
	}
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
		case SIM_SPEED_UP:
			keySpeedUp = 1;
			break;
		case SIM_SLOW_DOWN:
			keySlowDown = 1;
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
		case SIM_SPEED_UP:
			keySpeedUp = 0;
			break;
		case SIM_SLOW_DOWN:
			keySlowDown = 0;
			break;
		default:
			break;
	}
}

void updateValues(int n){
	// Frame limiter
	if(current_epoch < num_epochs){
		if (VISUALIZATION) glutTimerFunc(simSpeed, updateValues, 0);
	}
	else{
		if (AUTO_EXIT && num_steps > 1)
			exit(0);
		if (VISUALIZATION) glutTimerFunc(simSpeed, updateValues, 0);
	}


	num_steps++;
	if(num_steps == EPOCH_STEPS){
		num_steps = 0;

		// Error evaluation
		mlps.at(current_mlp).error = compute_error();

		// Write error to data file
		#if COLLECT_DATA
			distance_file_stream << mlps.at(current_mlp).error << ",";
			count_file_stream << count_robots_at_goal() << ",";
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
				//printf("R%d[%08.3f]: %s",i,mlps.at(i).error,str);
			}
			//printf("\nAVG: %f", sum/NUM_PARENTS);
			//printf("\n");

			// Erasing the worst mlps
			mlps.erase(mlps.begin()+NUM_PARENTS,mlps.end());

			// Reproducing the best mlps
			for(int i = 0; i < POP_SIZE-NUM_PARENTS; i++){

				// Parent selection
				Mlp mlp(&mlps.at(rand()%NUM_PARENTS));

				// Mutation
				mlp.mutate(mutation_range);

				// Inserting into population
				mlps.insert(mlps.end(), mlp);

			}
			#if COLLECT_DATA
				distance_file_stream << endl;
				count_file_stream << endl;
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

	// SimSpeed
	if(keySpeedUp)
		simSpeed /= 1.05;
	if(keySlowDown)
		simSpeed *= 1.05;
	simSpeed = clamp_val(simSpeed, 0.001, 300);

	// Save NN weights
	if(saveWeights || (autosave && current_epoch == num_epochs && current_mlp == 0 &&  num_steps == 1)){
		mlps.at(current_mlp).store(weights_file.c_str());
		saveWeights = 0;
	}

	if(loadWeights || (autoload && current_epoch == 0 && current_mlp == 0 &&  num_steps == 1)){
		for(auto &mlp: mlps)
			mlp.load(weights_file.c_str());
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
		r.update(weight, flags, blocks);

	// String printed in the screen corner
	sprintf(statusBuffer,"Robot Count: %02d Epoch: %06d/%06d MLP: %02d/%02d Steps: %03d/%03d Weight: %4.2f, Sim Speed: %4.2f",flock.size(),current_epoch,num_epochs,current_mlp,POP_SIZE,num_steps,EPOCH_STEPS,weight,1/simSpeed);

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
		glColor3f(0.0, 0.1, 0.25);
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

		// Goal flag
		for (auto const f : flags){
			int radius = sqrt(ATR_RADIUS);
			glColor3f(0.0, 0.1, 0.25);
			glBegin(GL_POLYGON);
			for(double i = 0; i < 2 * PI; i += PI / 24) //<-- Change this Value
					glVertex3f(cos(i) * radius - f.x, sin(i) * radius - f.y, 0.0);
			glEnd();
			// f.render(1,0);
		}

		// Drawing robots
		for(auto const &r : flock)
			r.render_robot();

		// Visualize radii
		#if VISUALIZE_RADII
			for (auto const r : flock){	
					if (r.leader){
					glBegin(GL_LINE_LOOP);
					glColor3f(0.0, 0.1, 0.25);
					for(double i = 0; i < 2 * PI; i += PI / 24) //<-- Change this Value
							glVertex3f(cos(i) * sqrt(ATR_RADIUS) - r.x, sin(i) * sqrt(ATR_RADIUS) - r.y, 0.0);
					glEnd();
					glBegin(GL_LINE_LOOP);
					glColor3f(0.1, 0.2, 0.4);
					for(double i = 0; i < 2 * PI; i += PI / 24) //<-- Change this Value	
							glVertex3f(cos(i) * sqrt(ORI_RADIUS) - r.x, sin(i) * sqrt(ORI_RADIUS) - r.y, 0.0);
					glEnd();
					glBegin(GL_LINE_LOOP);
					glColor3f(0.3, 0.3, 0.1);
					for(double i = 0; i < 2 * PI; i += PI / 24) //<-- Change this Value
							glVertex3f(cos(i) * sqrt(REP_RADIUS) - r.x, sin(i) * sqrt(REP_RADIUS) - r.y, 0.0);
					glEnd();
				}
			}
		#endif 

		// Obstacles
		for (auto const b : blocks){
			int radius = 16; // Obstacle Size
			glColor3f(0.8, 0.8, 1.0);
			glBegin(GL_POLYGON);
			for(double i = PI / 4; i < 2 * PI; i += PI / 2) //<-- Change this Value
				glVertex3f(cos(i) * radius - b.x, sin(i) * radius - b.y, 0.0);
			glEnd();
			// b.render(1,0);
		}

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
	glColor4f(0,0,0,0);
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
	glRasterPos2i(20,20);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glutBitmapString(GLUT_BITMAP_8_BY_13,statusBuffer);

	glutSwapBuffers();
}

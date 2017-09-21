#include "includes.hpp"
#include "Wired.hpp"
#include "Textured.hpp"
#include "Robot.hpp"

Textured cursor;
Textured flag;
vector<Robot> robot;

// Status
int iron			= 4500;
int power			= 3000;
double hqX			= 0;
double hqY			= 0;

double old_mouse_x	= 0;
double old_mouse_y	= 0;
double mouse_gnd_x	= 0;
double mouse_gnd_y	= 0;

// Screen Resolution
int window_w		= 0;
int window_h		= 0;

// Camera position/motion
double view_x		= 0;
double view_y		= 0;
double x_min		= 0;
double x_max		= 0;
double y_min		= 0;
double y_max		= 0;
double cam_speed	= 2;
double scn_scale	= SCN_SCALE;

// User input flags
bool mouse_l		= 0;
bool mouse_m		= 0;
bool mouse_r		= 0;
bool view_robot		= 0;

// Keyboard mov flags
bool keyLeft		= 0;
bool keyRight		= 0;
bool keyDown		= 0;
bool keyUp			= 0;
bool keyZoomIn		= 0;
bool keyZoomOut		= 0;

// Mouse mov flags
bool mouseLeft		= 0;
bool mouseRight		= 0;
bool mouseDown		= 0;
bool mouseUp		= 0;

bool selecting		= false;
double selX1		= 0;
double selY1		= 0;
double selX2		= 0;
double selY2		= 0;

char statusBuffer[BUFFER_SIZE];

void getScreenResolution(int& h, int& v);

void iniGl();

void areaSelect();

void singleSelect();

void mouseButton(int b,int s,int x,int y);

void cursorUpdate(int x,int y);

void mouseMove(int x, int y);

void mouseAction(int x, int y);

void glutMouseFunc(int button, int state, int x, int y);

void keyPressed(unsigned char key, int x, int y);

void keyReleased(unsigned char key, int x, int y);

void updateValues(int n);

bool checkCol(int index);

void RenderScene();

int main(int argc, char **argv){

	// Agent Shape
	vector<pair<double, double>> shape;
	shape.push_back(pair<double, double>(0.0,0.50));
	shape.push_back(pair<double, double>(1.0,0.75));
	shape.push_back(pair<double, double>(1.0,0.25));

	getScreenResolution(window_w,window_h);

	glutInit(&argc, argv);

	iniGl();

	view_x = hqX;
	view_y = hqY;

	if(cursor.init(0,0,64,64,0,"img/cursor.png")) return 0;
	if(flag.init(0,0,16,16,0,"img/flag.png")) return 0;

	// Spawning robots
	for(int i = 0; i < NUM_ROBOTS; i++){
		robot.push_back(Robot());
		robot.at(robot.size()-1).init(hqX-rand()%SPAWN_RANGE+SPAWN_RANGE/2,hqY-rand()%SPAWN_RANGE+SPAWN_RANGE/2,8,8,0,ROBOT_VEL,shape);
		robot.at(robot.size()-1).respawn(hqX-rand()%SPAWN_RANGE+SPAWN_RANGE/2,hqY-rand()%SPAWN_RANGE+SPAWN_RANGE/2);
	}

	glutMainLoop();

	return 0;
}

// Get the horizontal and vertical screen sizes in pixel
void getScreenResolution(int& h, int& v){
	Display* d = XOpenDisplay(NULL);
	Screen*  s = DefaultScreenOfDisplay(d);
	h   = s->width;
	v   = s->height;
}

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
	if (FULL_SCREEN)
		glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);
}

void areaSelect(){
	double minX = (selX2 > selX1)?selX1:selX2;
	double maxX = (selX2 > selX1)?selX2:selX1;
	double minY = (selY2 > selY1)?selY1:selY2;
	double maxY = (selY2 > selY1)?selY2:selY1;
	for(int i = 0; i < robot.size(); i++){
		if(robot.at(i).wire.x-robot.at(i).wire.w/2 < minX) continue;
		if(robot.at(i).wire.x+robot.at(i).wire.w/2 > maxX) continue;
		if(robot.at(i).wire.y-robot.at(i).wire.h/2 < minY) continue;
		if(robot.at(i).wire.y+robot.at(i).wire.h/2 > maxY) continue;
		robot.at(i).selected = true;
	}
}

void singleSelect(){
	for(int i = 0; i < robot.size(); i++){
		if(robot.at(i).wire.x-robot.at(i).wire.w/2 > selX1){robot.at(i).selected = false;continue;}
		if(robot.at(i).wire.x+robot.at(i).wire.w/2 < selX1){robot.at(i).selected = false;continue;}
		if(robot.at(i).wire.y-robot.at(i).wire.h/2 > selY1){robot.at(i).selected = false;continue;}
		if(robot.at(i).wire.y+robot.at(i).wire.h/2 < selY1){robot.at(i).selected = false;continue;}
		robot.at(i).selected = true;
	}
}

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
				double candidateX = mouse_gnd_x;
				double candidateY = mouse_gnd_y;
				for(int i = 0; i < robot.size(); i++)
					if(robot.at(i).selected)
						robot.at(i).setRef(candidateX,candidateY);
				flag.x = mouse_gnd_x;
				flag.y = mouse_gnd_y;
			}
			else
				mouse_r = 0;
			break;
		default:
			break;
	}
}

double scn2space(int a){
	return scn_scale*((double)a);
}

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

// When mouse clicks
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
		case VIEW_HQ:
			view_x = hqX;
			view_y = hqY;
			break;
		case VIEW_ROBOT:
			view_robot = !view_robot;
			break;
		case RESET_ZOOM:
			scn_scale = SCN_SCALE;
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

	sprintf(statusBuffer,"R: %02d I: %08d P: %08d V: %01d",robot.size(),iron/100,power/100,view_robot);

	// Frame limiter
	glutTimerFunc(FRAME_TIME,updateValues,0);

	if(view_robot){
		for(int i = 0; i < robot.size(); i++)
			if(robot.at(i).selected){
				view_x = robot.at(i).wire.x;
				view_y = robot.at(i).wire.y;
				break;
			}
	}

	// Moving the camera given the camera speed
	if(mouseLeft	|	keyLeft)	view_x += cam_speed;
	if(mouseRight	|	keyRight)	view_x -= cam_speed;
	if(mouseDown	|	keyDown)	view_y += cam_speed;
	if(mouseUp		|	keyUp)		view_y -= cam_speed;

	if(keyZoomIn)
		scn_scale /= 1.05;
	if(keyZoomOut)
		scn_scale *= 1.05;

	// Updating camera projection parameters
	x_min = -view_x - scn_scale*window_w/2;
	x_max = -view_x + scn_scale*window_w/2;
	y_min = -view_y - scn_scale*window_h/2;
	y_max = -view_y + scn_scale*window_h/2;

	for(int i = 0; i < robot.size(); i++)
		robot.at(i).update(checkCol(i));

}

bool checkCol(int index){
	for(int i = 0; i < robot.size(); i++)
		if(i != index){
			if(robot.at(index).wire.x+robot.at(index).wire.w/2 < robot.at(i).wire.x-robot.at(i).wire.w/2) continue;
			if(robot.at(index).wire.x-robot.at(index).wire.w/2 > robot.at(i).wire.x+robot.at(i).wire.w/2) continue;
			if(robot.at(index).wire.y+robot.at(index).wire.h/2 < robot.at(i).wire.y-robot.at(i).wire.h/2) continue;
			if(robot.at(index).wire.y-robot.at(index).wire.h/2 > robot.at(i).wire.y+robot.at(i).wire.h/2) continue;
			return true;
		}	
	return false;	
}

void RenderScene(){
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Space coordinates
	glPushMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(x_min,x_max,y_min,y_max);
		glMatrixMode(GL_MODELVIEW);

		for(int i = 0; i < robot.size(); i++)
			robot.at(i).render();
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

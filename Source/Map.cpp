#include "Map.hpp"

Map::Map(){
	this->w = 0;
	this->h = 0;
	this->cols = 1;
	this->rows = 1;
};

Map::~Map(){
	#if PATH_DEBUG
		delete [] pathCost;
		delete [] visited;
	#endif
	delete map;
};

int Map::init(double w,double h,char * filename){
	this->w = w;
	this->h = h;
	this->cols = 1;
	this->rows = 1;
	bool returnVal = 0;
	returnVal |= loadMap(filename);
	returnVal |= loadGLTexture("Images/grass.png",		0);
	returnVal |= loadGLTexture("Images/obstacle.png",	1);
	returnVal |= loadGLTexture("Images/ore.png",		2);
	returnVal |= loadGLTexture("Images/crystal.png",	3);
	returnVal |= loadGLTexture("Images/hq.png",			4);
	#if PATH_DEBUG
		pathCost	= new float[cols*rows];
		visited		= new bool[cols*rows];
		for(int i = 0; i < rows*cols; i++){
			pathCost[i] = (float)INT_MAX;
			visited[i]	= false;
		}
	#endif
	return returnVal;
};

int Map::checkCol(double x,double y,double w,double h){
	int i = 0;
	int j = 0;
	i = (int)ceil((y+h/2)/this->h+rows/2);
	j = (int)ceil((x+w/2)/this->w+cols/2);
	if(i >= 0 && j >= 0 && i < rows && j < cols && map[coordinate2index(i,j)]) return map[coordinate2index(i,j)];
	i = (int)ceil((y-h/2)/this->h+rows/2);
	j = (int)ceil((x+w/2)/this->w+cols/2);
	if(i >= 0 && j >= 0 && i < rows && j < cols && map[coordinate2index(i,j)]) return map[coordinate2index(i,j)];
	i = (int)ceil((y+h/2)/this->h+rows/2);
	j = (int)ceil((x-w/2)/this->w+cols/2);
	if(i >= 0 && j >= 0 && i < rows && j < cols && map[coordinate2index(i,j)]) return map[coordinate2index(i,j)];
	i = (int)ceil((y-h/2)/this->h+rows/2);
	j = (int)ceil((x-w/2)/this->w+cols/2);
	if(i >= 0 && j >= 0 && i < rows && j < cols && map[coordinate2index(i,j)]) return map[coordinate2index(i,j)];
	return 0;
}

int Map::nearestEmptyPos(double *x,double *y){
	int i = 0;
	int j = 0;
	pos2index(*x,*y,&i,&j);
	if(checkIndex(i,j) < 0)
		return -1;
	if(map[coordinate2index(i,j)] == 0)
		return 0;
	int iA = 0;
	int jA = 0;
	int index = 0;
	iA = i+1;
	jA = j+0;
	index = coordinate2index(iA,jA);
	if(checkIndex(iA,jA) >= 0 && map[index] == 0){
		index2pos(iA,jA,x,y);
		(*y) -= 0.49*h;
		return 0;
	}
	iA = i+0;
	jA = j+1;
	index = coordinate2index(iA,jA);
	if(checkIndex(iA,jA) >= 0 && map[index] == 0){
		index2pos(iA,jA,x,y);
		(*x) -= 0.49*w;
		return 0;
	}
	iA = i-1;
	jA = j-0;
	index = coordinate2index(iA,jA);
	if(checkIndex(iA,jA) >= 0 && map[index] == 0){
		index2pos(iA,jA,x,y);
		(*y) += 0.49*h;
		return 0;
	}
	iA = i-0;
	jA = j-1;
	index = coordinate2index(iA,jA);
	if(checkIndex(iA,jA) >= 0 && map[index] == 0){
		index2pos(iA,jA,x,y);
		(*x) += 0.49*w;
		return 0;
	}
	return -1;
}

void Map::getHQ(double &x,double &y){
	for(int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++)
			if(map[coordinate2index(i,j)] == 4){
				x = j*w-(cols+1)*w/2;
				y = i*h-(rows+1)*h/2;
				return;
			}
}

void Map::pos2index(double x,double y,int *i,int *j){
	(*i) = (int)ceil(y/h + rows/2);
	(*j) = (int)ceil(x/w + cols/2);
}

void Map::index2pos(int i,int j,double *x,double *y){
	(*x) = -(cols*w/2-j*w + 0.5*w);
	(*y) = -(rows*h/2-i*h + 0.5*w);		
}

int Map::coordinate2index(int i,int j){
	return i*cols+j;
}

void Map::index2coordinate(int index,int *i,int *j){
	*j = index%cols;
	*i = (index - (*j))/cols;
}

// Ensure index is valid
// -1: Index out of map
//  0: Index valid
int Map::checkIndex(int i,int j){
	if(i < 0 || j < 0 || i >= rows || j >= cols)
		return -1;
	return coordinate2index(i,j);
}

// Ensure corner is free
// -1: corner blocked
//  0: corner free
int Map::checkCorner(int iC,int i,int jC,int j){
	if(abs(i)+abs(j) == 2 && (map[coordinate2index(iC+i,jC)] || map[coordinate2index(iC,jC+j)]))
		return -1;
	return 0;
}

// Admissible heuristic (underestimate cost to goal) 
float Map::distanceHeuristic(int i,int j,int iF,int jF){
	double x = 0;
	double y = 0;
	double xF = 0;
	double yF = 0;
	index2pos(i,j,&x,&y);
	index2pos(iF,jF,&xF,&yF);
	return sqrt(pow(xF-x,2) + pow(yF-y,2))/w;
}

int Map::path(double xI,double yI,double xF,double yF,vector<double> *xP,vector<double> *yP){

	xP->clear();
	yP->clear();

	int iI = 0;
	int jI = 0;
	int iF = 0;
	int jF = 0;

	// Converting from continuous positions to graph indexes
	pos2index(xI,yI,&iI,&jI);
	pos2index(xF,yF,&iF,&jF);

	// Memory allocation
	vector<int> *iP = new vector<int>;
	vector<int> *jP = new vector<int>;

	// Path search
	if(search(iI,jI,iF,jF,iP,jP) == 0) return 0;

	// Converting from graph indexes to continuous positions
	double auxX = 0;
	double auxY = 0;
	for(int i = 0; i < iP->size(); i++){
		index2pos(iP->at(i),jP->at(i),&auxX,&auxY);
		xP->push_back(auxX);
		yP->push_back(auxY);
	}
	delete iP;
	delete jP;

	return 1;
}

// Expand adjacent nodes
void Map::expand(Node *parent,int iC,int jC,int iF,int jF,int i,int j,float cost){

	// Checking if node valid
	int index = checkIndex(iC+i,jC+j);
	if(index < 0) return;
	if(map[index]) return;
	if(checkCorner(iC,i,jC,j) < 0) return;
	for(Node* node : closed)
		if(index == node->index) return;

	for(Node* node : frontier)
		if(index == node->index){
			if(parent->cost+cost > node->cost)
				return;
			else{
				frontier.remove(node);
				break;
			}
		}

	// Heuristic scaling
	float heuristic = INFLATION*distanceHeuristic(iC+i,jC+j,iF,jF);

	// Creating node
	Node *node = new Node(index,parent->cost+cost,heuristic,parent);

	// Inserting elements to the sorted list
	list<Node*>::iterator it = frontier.begin();
	while(it != frontier.end()){
		if((*it)->est_cost > node->est_cost)
			break;
		it++;
	}
	frontier.insert(it,node);
}

int Map::search(int iI,int jI,int iF,int jF,vector<int> *iP,vector<int> *jP){

	// Clearing lists
	for(Node* node : frontier)
		delete node;
	for(Node* node : closed)
		delete node;
	frontier.clear();
	closed.clear();

	// Starting and Goal Nodes Indexes
	Node *nodeI = new Node(coordinate2index(iI,jI),0,0,NULL);
	int indexF = coordinate2index(iF,jF);

	// Initializing frontier list
	frontier.push_back(nodeI);

	// Aux variables
	int iC = 0;
	int jC = 0;
	Node *nodeC = NULL;

	// Search loop
	for(;;){

		// Checking if frontier empty (failure)
		if(frontier.empty()) return 0;

		// Visiting current node (least cost)
		nodeC = frontier.front();
		frontier.pop_front();

		// Adding current node to the closed list
		closed.push_back(nodeC);

		// Checking if goal found
		if(nodeC->index == indexF) break;

		// Expanding current node
		index2coordinate(nodeC->index,&iC,&jC);
		expand(nodeC,iC,jC,iF,jF,+1,+0,1.00);
		expand(nodeC,iC,jC,iF,jF,+0,+1,1.00);
		expand(nodeC,iC,jC,iF,jF,-1,+0,1.00);
		expand(nodeC,iC,jC,iF,jF,+0,-1,1.00);
		expand(nodeC,iC,jC,iF,jF,+1,+1,1.41);
		expand(nodeC,iC,jC,iF,jF,-1,+1,1.41);
		expand(nodeC,iC,jC,iF,jF,-1,-1,1.41);
		expand(nodeC,iC,jC,iF,jF,+1,-1,1.41);
	}

	// Final path position
	iP->push_back(iF);
	jP->push_back(jF);

	// Defining path as a sequence of positions
	for(;;){

		// Check if path completed
		if(nodeC->index == nodeI->index) break;

		// Moving to parent node
		nodeC = nodeC->parent;
		index2coordinate(nodeC->index,&iC,&jC);
		
		// Inserting position in the path vector
		iP->push_back(iC);
		jP->push_back(jC);
	}

	// Collecting cost and visited flags for visualization
	#if PATH_DEBUG
		for(int i = 0; i < rows*cols; i++){
			pathCost[i] = (float)INT_MAX;
			visited[i]	= false;
		}
		for(Node* node : frontier)
			pathCost[node->index] = node->cost;
		for(Node* node : closed){
			pathCost[node->index] = node->cost;
			visited[node->index] = true;
		}
	#endif

	return 1;

}

void Map::render(){
	glEnable(GL_TEXTURE_2D);
		glPushMatrix();
			glColor3f(1.0,1.0,1.0);
			glTranslatef(cols*w/2,rows*h/2,0);
			for(int i = 0; i < rows; i++)
				for(int j = 0; j < cols; j++){
					glBindTexture(GL_TEXTURE_2D,texture[map[coordinate2index(i,j)]]);
					glPushMatrix();
						glTranslatef(-j*w,-i*h,0);
						glScalef(w,h,1);
						glBegin(GL_QUADS);
							glTexCoord2f(0,0);
							glVertex2f(0,0);
							glTexCoord2f(0,1);
							glVertex2f(0,1);
							glTexCoord2f(1,1);
							glVertex2f(1,1);
							glTexCoord2f(1,0);
							glVertex2f(1,0);
						glEnd();
					glPopMatrix();
				}
		glPopMatrix();
	glDisable(GL_TEXTURE_2D);

	#if PATH_DEBUG
		char numBuffer[BUFFER_SIZE];
		glPushMatrix();
			glTranslatef(cols*w/2,rows*h/2,0);
			for(int i = 0; i < rows; i++)
				for(int j = 0; j < cols; j++){
					if(pathCost[coordinate2index(i,j)] > 999)
						sprintf(numBuffer,"oo");
					else
						sprintf(numBuffer,"%.1f",pathCost[coordinate2index(i,j)]);
					glColor3f(1,1,(1-visited[coordinate2index(i,j)]));
					glPushMatrix();
						glTranslatef(w/2-j*w,h/2-i*h,0);
						glRasterPos2i(-2,0);
						glScalef(w/3,h/3,1);
						glColor4f(0,0,0,0.75);
						glBegin(GL_QUADS);
							glVertex2f(-1,-1);
							glVertex2f(-1,+1);
							glVertex2f(+1,+1);
							glVertex2f(+1,-1);
						glEnd();
						glutBitmapString(GLUT_BITMAP_8_BY_13,numBuffer);
					glPopMatrix();
				}
		glPopMatrix();
	#endif
};

int Map::loadMap(char *filename){

	FILE *file;	
	char fileBuffer[BUFFER_SIZE]; 
	int aux = 0;

	// Checking if origin file exists
	file 	= fopen(filename,"r");
	if(file == NULL){
		cout << "Error: Origin file '" << filename << "' not found." << endl;
		return 1;
	}

	// Counting cols
	if(fgets(fileBuffer,BUFFER_SIZE,file) == NULL){
		cout << "Error: Error while reading file." << endl;
		return 1;
	}
	while(fileBuffer[aux] != '\n'){
		if(fileBuffer[aux] == ',')
			cols++;
		aux++;
	}

	// Counting rows
	while(fgets(fileBuffer,BUFFER_SIZE,file) != NULL) rows++;

	rewind(file);

	map = new int[cols*rows];

	// for(int i = rows-1; i >= 0; i--){
	for(int i = 0; i < rows; i++){

		// Load line
		fgets(fileBuffer,BUFFER_SIZE,file);

		aux = 0;
		for(int j = cols-1; j >= 0; j--){

			map[coordinate2index(i,j)] = atoi(fileBuffer+aux);

			// Seek next value
			while(fileBuffer[aux] != ',')
				if(fileBuffer[aux] == '\n')
					break;
				else
					aux++;
			aux++;
		}

	}

	fclose(file);
	return 0;

}

int Map::loadGLTexture(char *filename,int layer){

	// header for testing if it is a png
	png_byte header[8];

	// open file as binary
	FILE *fp = fopen(filename,"rb");
		if(!fp){
		return 1;
	}

	// read the header
	fread(header, 1, 8, fp);

	// test if png
	int is_png = !png_sig_cmp(header,0,8);
	if(!is_png){
		fclose(fp);
		return 1;
	}

	// create png struct
	png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
	NULL, NULL);
	if(!png_ptr){
		fclose(fp);
		return 1;
	}

	// create png info struct
	png_infop info_ptr = png_create_info_struct(png_ptr);
	if(!info_ptr){
		png_destroy_read_struct(&png_ptr, (png_infopp) NULL, (png_infopp) NULL);
		fclose(fp);
		return 1;
	}

	// create png info struct
	png_infop end_info = png_create_info_struct(png_ptr);
	if(!end_info){
		png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp) NULL);
		fclose(fp);
		return 1;
	}

	// png error stuff, not sure libpng man suggests this.
	if(setjmp(png_jmpbuf(png_ptr))){
		png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
		fclose(fp);
		return 1;
	}

	// init png reading
	png_init_io(png_ptr, fp);

	// let libpng know you already read the first 8 bytes
	png_set_sig_bytes(png_ptr, 8);

	// read all the info up to the image data
	png_read_info(png_ptr, info_ptr);

	// variables to pass to get info
	int bit_depth, color_type;
	png_uint_32 twidth, theight;

	// get info about png
	png_get_IHDR(png_ptr, info_ptr, &twidth, &theight, &bit_depth, &color_type,
	NULL, NULL, NULL);

	// update width and height based on png info
	int width = twidth;
	int height = theight;

	// Update the png info struct.
	png_read_update_info(png_ptr,info_ptr);

	// Row size in bytes.
	int rowbytes = png_get_rowbytes(png_ptr, info_ptr);

	// Allocate the image_data as a big block, to be given to opengl
	png_byte *image_data = new png_byte[rowbytes * height];
	if(!image_data){
		//clean up memory and close stuff
		png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
		fclose(fp);
		return 1;
	}

	//row_pointers is for pointing to image_data for reading the png with libpng
	png_bytep *row_pointers = new png_bytep[height];
	if(!row_pointers){
		//clean up memory and close stuff
		png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
		delete[] image_data;
		fclose(fp);
		return 1;
	}

	// set the individual row_pointers to point at the correct offsets of image_data
	for(int i=0; i < height; ++i)
		row_pointers[height - 1 - i] = image_data + i * rowbytes;

	// read the png into image_data through row_pointers
	png_read_image(png_ptr,row_pointers);

	glGenTextures(1,&texture[layer]);
	glBindTexture(GL_TEXTURE_2D, texture[layer]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE,(GLvoid*) image_data);

	//clean up memory and close stuff
	png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
	delete[] image_data;
	delete[] row_pointers;
	fclose(fp);

	return 0;
};

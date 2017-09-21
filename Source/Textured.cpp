#include "Textured.hpp"

Textured::Textured(){
	this->x = 0;
	this->y = 0;
	this->w = 0;
	this->h = 0;
	this->t = 0;
};

Textured::~Textured(){};

int Textured::init(double x,double y,double w,double h,double t,char * filename){
	this->x = x;
	this->y = y;
	this->w = w;
	this->h = h;
	this->t = t;
	return loadGLTextures(filename);
};

void Textured::update(double x,double y,double w,double h,double t){
	this->x = x;
	this->y = y;
	this->w = w;
	this->h = h;
	this->t = t;
};

void Textured::render(bool global,bool highlight){
		glPushMatrix();
			glEnable(GL_TEXTURE_2D);
			glColor3f(1.0,1.0,1.0);
			if(global){
				glTranslatef(-x,-y,0);
				glScalef(w,h,1);
			}
			else{
				glTranslatef(x,y,0);
				glScalef(w,-h,1);
			}
			glRotatef(t,0,0,1);
			glTranslatef(-0.5,-0.5,0);
			glColor3f(1.0,1.0,1.0);
			glBindTexture(GL_TEXTURE_2D,texture[0]);
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
			glDisable(GL_TEXTURE_2D);
			if(highlight){
				glLineWidth(3);
				glColor4f(1,0,0,0.75);
				glBegin(GL_LINE_LOOP);
					glVertex2f(0,0);
					glVertex2f(0,1);
					glVertex2f(1,1);
					glVertex2f(1,0);
				glEnd();
				glLineWidth(1);
			}
		glPopMatrix();
};

int Textured::loadGLTextures(char *filename){

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

	glGenTextures(1,&texture[0]);
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE,(GLvoid*) image_data);

	//clean up memory and close stuff
	png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
	delete[] image_data;
	delete[] row_pointers;
	fclose(fp);

	return 0;
};

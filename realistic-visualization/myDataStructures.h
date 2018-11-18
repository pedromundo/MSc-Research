#ifndef MY_DATA_STRUCTURES
#define MY_DATA_STRUCTURES 1

#include <vector>
#include <GL/gl.h>

using namespace std;

typedef struct {
	GLfloat r;
	GLfloat g;
	GLfloat b;
	GLfloat a;
} Color;

typedef struct {	
	GLfloat u;
	GLfloat v;
} TexCoord;

typedef struct {
	GLfloat nx;
	GLfloat ny;
	GLfloat nz;
} Normal;

typedef struct {
	GLfloat x;
	GLfloat y;
	GLfloat z;
	Normal normal;
	TexCoord uv;	
	glm::vec3 tan;
	glm::vec3 bin;
} Vertex;

typedef struct {	
	GLuint f1;	
	GLuint f2;
	GLuint f3;
} Face;

#define printOpenGLError() printOglError(__FILE__, __LINE__)

int printOglError(char *file, int line)
{

	GLenum glErr;
	int    retCode = 0;

	glErr = glGetError();
	if (glErr != GL_NO_ERROR)
	{
		printf("glError in file %s @ line %d: %s\n",
			file, line, gluErrorString(glErr));
		retCode = 1;
	}
	return retCode;
}

#endif

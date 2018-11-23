//OpenGL Stuff
#include <GL/glew.h>
#include <GL/glut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//My includes
#include <cstdio>
#include <SOIL/SOIL.h>
#include "myDataStructures.h"
#include "initShaders.h"
#include "rply.h"
#include "rplyfile.h"

GLvoid reshape(GLint x, GLint y);

GLboolean g_bExitESC = false, g_bRotateModel = false;

//Window Dimensions
GLuint wWidth = 1024, wHeight = 768;
//Shader uniforms
GLuint lightDiffusePower = 100, lightSpecularPower = 10, lightDistance = 10;
//# of vertices and tris
GLulong nvertices, ntriangles;
//Texture properties
GLint wTex, hTex, cTex, wNor, hNor, cNor;
//Handlers for the VBOs, FBOs, texArrays shader programs
GLuint VertexArrayIDs[1], vertexbuffers[2], textureArrays[5], basicShader;
GLfloat fov = 60.0f, direct_mapping_step = 90.0f;
std::size_t vertexSize = (3 * sizeof(GLfloat) + 3 * sizeof(GLfloat) + 2 * sizeof(GLfloat) + 2 * sizeof(glm::vec3));
//MVP Matrices
glm::mat4 Projection, View, Model;
glm::vec3 eyePos = glm::vec3(1.5, 1.5, 1.5);
glm::vec3 lightPos = glm::vec3(1.5, 1.5, 1.5);

std::vector<Vertex> *vertices = new std::vector<Vertex>();
std::vector<Face> *faces = new std::vector<Face>();

GLubyte *texture_0;
GLubyte *texture_90;
GLubyte *texture_180;
GLubyte *texture_270;
GLubyte *normalmap;

void computeTangents()
{
	for (GLint i = 0; i < faces->size(); ++i)
	{
		// Shortcuts for vertices
		Vertex a = vertices->at(faces->at(i).f1);
		Vertex b = vertices->at(faces->at(i).f2);
		Vertex c = vertices->at(faces->at(i).f3);

		glm::vec3 v0 = glm::vec3(a.x, a.y, a.z);
		glm::vec3 v1 = glm::vec3(b.x, b.y, b.z);
		glm::vec3 v2 = glm::vec3(c.x, c.y, c.z);

		// Shortcuts for UVs
		glm::vec2 uv0 = glm::vec2(a.uv.u, a.uv.v);
		glm::vec2 uv1 = glm::vec2(b.uv.u, b.uv.v);
		glm::vec2 uv2 = glm::vec2(c.uv.u, c.uv.v);

		// Edges of the triangle : postion delta
		glm::vec3 deltaPos1 = v1 - v0;
		glm::vec3 deltaPos2 = v2 - v0;

		// UV delta
		glm::vec2 deltaUV1 = uv1 - uv0;
		glm::vec2 deltaUV2 = uv2 - uv0;
		GLfloat r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
		glm::vec3 tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r;
		glm::vec3 bitangent = (deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x) * r;
		// Set the same tangent for all three vertices of the triangle.
		vertices->at(faces->at(i).f1).tan = tangent;
		vertices->at(faces->at(i).f2).tan = tangent;
		vertices->at(faces->at(i).f3).tan = tangent;
		// Same thing for binormals
		vertices->at(faces->at(i).f3).bin = bitangent;
		vertices->at(faces->at(i).f1).bin = bitangent;
		vertices->at(faces->at(i).f2).bin = bitangent;
	}
}

GLvoid shaderPlumbing()
{
	printOpenGLError();
	glPointSize(1);

	//MVP matrix
	GLuint MVPId = glGetUniformLocation(basicShader, "MVP");
	glUniformMatrix4fv(MVPId, 1, GL_FALSE, glm::value_ptr(Projection * View * Model));
	//MV matrix
	GLuint MVId = glGetUniformLocation(basicShader, "MV");
	glUniformMatrix3fv(MVId, 1, GL_FALSE, glm::value_ptr(glm::mat3(View * Model)));
	printOpenGLError();
	//V Matrix
	GLuint MId = glGetUniformLocation(basicShader, "M");
	glUniformMatrix3fv(MId, 1, GL_FALSE, glm::value_ptr(glm::mat3(Model)));
	printOpenGLError();
	//Eye Position
	GLuint eyePosId = glGetUniformLocation(basicShader, "eyePos");
	glUniform3f(eyePosId, eyePos.x, eyePos.y, eyePos.z);
	printOpenGLError();
	//Diffuse lighting intensity
	GLuint lightDiffusePowerId = glGetUniformLocation(basicShader, "lightDiffusePower");
	glUniform1ui(lightDiffusePowerId, lightDiffusePower);
	printOpenGLError();
	//Specular lighting intensity
	GLuint lightSpecularPowerId = glGetUniformLocation(basicShader, "lightSpecularPower");
	glUniform1ui(lightSpecularPowerId, lightSpecularPower);
	printOpenGLError();
	//Light distance for intensity calculations
	GLuint lightDistanceId = glGetUniformLocation(basicShader, "lightDistance");
	glUniform1ui(lightDistanceId, lightDistance);
	printOpenGLError();
	//Light position
	GLuint lightID = glGetUniformLocation(basicShader, "lightPos");
	glUniform3f(lightID, lightPos.x, lightPos.y, lightPos.z);
	printOpenGLError();

	glBindVertexArray(VertexArrayIDs[0]);
	//Vertex attributes
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[0]);
	glEnableVertexAttribArray(glGetAttribLocation(basicShader, "appPosition_modelspace"));
	glVertexAttribPointer(glGetAttribLocation(basicShader, "appPosition_modelspace"), 3, GL_FLOAT, GL_FALSE, vertexSize, (GLvoid *)0);
	glEnableVertexAttribArray(glGetAttribLocation(basicShader, "appNormal_modelspace"));
	glVertexAttribPointer(glGetAttribLocation(basicShader, "appNormal_modelspace"), 3, GL_FLOAT, GL_FALSE, vertexSize, (const GLvoid *)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(glGetAttribLocation(basicShader, "appTexCoord"));
	glVertexAttribPointer(glGetAttribLocation(basicShader, "appTexCoord"), 2, GL_FLOAT, GL_FALSE, vertexSize, (const GLvoid *)(6 * sizeof(GLfloat)));
	glEnableVertexAttribArray(glGetAttribLocation(basicShader, "appTangent_modelspace"));
	glVertexAttribPointer(glGetAttribLocation(basicShader, "appTangent_modelspace"), 3, GL_FLOAT, GL_FALSE, vertexSize, (const GLvoid *)(8 * sizeof(GLfloat)));
	glEnableVertexAttribArray(glGetAttribLocation(basicShader, "appBinormal_modelspace"));
	glVertexAttribPointer(glGetAttribLocation(basicShader, "appBinormal_modelspace"), 3, GL_FLOAT, GL_FALSE, vertexSize, (const GLvoid *)(11 * sizeof(GLfloat)));
	//Element vertex IDs
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertexbuffers[1]);
	printOpenGLError();
}

GLvoid display(GLvoid)
{
	printOpenGLError();
	glClearColor(0.3f, 0.3f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	shaderPlumbing();
	glDrawElements(GL_TRIANGLES, 3 * ntriangles, GL_UNSIGNED_INT, (void *)0);
	printOpenGLError();

	glutSwapBuffers();
	glutPostRedisplay();

	//Unbinding stuff
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(glGetAttribLocation(basicShader, "appPosition_modelspace"));
	glDisableVertexAttribArray(glGetAttribLocation(basicShader, "appNormal_modelspace"));
	glDisableVertexAttribArray(glGetAttribLocation(basicShader, "appTexCoord"));
	glDisableVertexAttribArray(glGetAttribLocation(basicShader, "appTangent_modelspace"));
	glDisableVertexAttribArray(glGetAttribLocation(basicShader, "appBinormal_modelspace"));
}

GLvoid initShaders()
{
	basicShader = InitShader("basicShader.vert", "basicShader.frag");
	glUseProgram(basicShader);
	printOpenGLError();
}

GLvoid keyboard(GLubyte key, GLint x, GLint y)
{
	switch (key)
	{
	case 27:
		g_bExitESC = true;
#if defined(__APPLE__) || defined(MACOSX)
		exit(EXIT_SUCCESS);
#else
		glutDestroyWindow(glutGetWindow());
		return;
#endif
		break;
	case 'r':
	case 'R':
		g_bRotateModel = !g_bRotateModel;
		break;
	case '=':
	case '+':
		fov -= 2.0f;
		Projection = glm::perspective(glm::radians(fov), (GLfloat)wWidth / (GLfloat)wHeight, 0.1f, 100.0f);
		break;
	case '-':
	case '_':
		fov += 2.0f;
		Projection = glm::perspective(glm::radians(fov), (GLfloat)wWidth / (GLfloat)wHeight, 0.1f, 100.0f);
		break;
	case 'l':
	case 'L':
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;
	case 'p':
	case 'P':
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	default:
		break;
	}
}

GLvoid reshape(GLint x, GLint y)
{
	wWidth = x;
	wHeight = y;
	glViewport(0, 0, x, y);
	glutPostRedisplay();
}

GLvoid process(GLvoid)
{
	if (g_bRotateModel)
	{
		View = glm::rotate(View, -0.005f, glm::vec3(0.0, 1.0, 0.0));
	}
}

GLint initGL(GLint *argc, GLchar **argv)
{
	glutInit(argc, argv);
	glutIdleFunc(process);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(wWidth, wHeight);
	glutCreateWindow("Direct Mapping");
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutReshapeFunc(reshape);
	glewInit();
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	return 1;
}

inline GLfloat interpolate(const GLfloat a, const GLfloat b, const GLfloat coefficient)
{
	return a + coefficient * (b - a);
}

Vertex tempPoint;
GLint vertex_cb(p_ply_argument argument)
{
	long currItem;
	ply_get_argument_user_data(argument, NULL, &currItem);
	if (currItem == 0)
	{
		tempPoint.x = ply_get_argument_value(argument);
	}
	else if (currItem == 1)
	{
		tempPoint.y = ply_get_argument_value(argument);
	}
	else if (currItem == 2)
	{
		tempPoint.z = ply_get_argument_value(argument);
	}
	else if (currItem == 3)
	{
		tempPoint.normal.nx = ply_get_argument_value(argument);
	}
	else if (currItem == 4)
	{
		tempPoint.normal.ny = ply_get_argument_value(argument);
	}
	else if (currItem == 5)
	{
		tempPoint.normal.nz = ply_get_argument_value(argument);

		//Finishing up the face with extra properties
		glm::vec3 front = glm::vec3(0.0, 0.0, 1.0); //assume that all the objects face Z+
		glm::vec3 vertex_xzdir = glm::normalize(glm::vec3(tempPoint.x, 0.0, tempPoint.z));
		GLfloat vertex_angle = glm::degrees(glm::acos(glm::dot(front, vertex_xzdir)));
		vertex_angle = tempPoint.x < 0 ? 360 - vertex_angle : vertex_angle;

		if((vertex_angle >= 0 && vertex_angle <= 45) || (vertex_angle >= 315 && vertex_angle <= 360)){
			//Quadrante frontal
			tempPoint.uv.u = tempPoint.x/3.000+0.475;
			tempPoint.uv.v = -tempPoint.y/2.250+0.535;
		}else if(vertex_angle > 45 && vertex_angle <= 135){
			//Quadrante lateral esquerdo
			tempPoint.uv.u = tempPoint.z/3.000+0.500;
			tempPoint.uv.v = -tempPoint.y/2.250+0.528;
		}else if(vertex_angle >= 135 && vertex_angle <= 225){
			//Quadrante traseiro
			tempPoint.uv.u = -tempPoint.x/3.000+0.475;
			tempPoint.uv.v = -tempPoint.y/2.250+0.535;
		}else if(vertex_angle > 225 && vertex_angle < 315){
			//Quadrante lateral direito
			tempPoint.uv.u = -tempPoint.z/3.000+0.500;
			tempPoint.uv.v = -tempPoint.y/2.250+0.550;
		}

		vertices->push_back(tempPoint);
	}
	return 1;
}

Face tempFace;
GLint face_cb(p_ply_argument argument)
{
	long length, value_index, currItem;
	ply_get_argument_property(argument, NULL, &length, &value_index);
	ply_get_argument_user_data(argument, NULL, &currItem);
	if (length == 3)
	{
		switch (value_index)
		{
		case 0:
			tempFace.f1 = ply_get_argument_value(argument);
			break;
		case 1:
			tempFace.f2 = ply_get_argument_value(argument);
			break;
		case 2:
			tempFace.f3 = ply_get_argument_value(argument);
			faces->push_back(tempFace);
			break;
		default:
			break;
		}
	}
	return 1;
}

GLvoid initTextures()
{
	//Texture data
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureArrays[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, wTex, hTex, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_0);
	glUniform1i(glGetUniformLocation(basicShader, "tex_0"), 0);

	//Texture data
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureArrays[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, wTex, hTex, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_90);
	glUniform1i(glGetUniformLocation(basicShader, "tex_90"), 1);

	//Texture data
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, textureArrays[2]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, wTex, hTex, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_180);
	glUniform1i(glGetUniformLocation(basicShader, "tex_180"), 2);

	//Texture data
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, textureArrays[3]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, wTex, hTex, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_270);
	glUniform1i(glGetUniformLocation(basicShader, "tex_270"), 3);

	//Normal data
	glActiveTexture(GL_TEXTURE4);
	glBindTexture(GL_TEXTURE_2D, textureArrays[4]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, wNor, hNor, 0, GL_RGB, GL_UNSIGNED_BYTE, normalmap);
	glUniform1i(glGetUniformLocation(basicShader, "nor"), 4);
	printOpenGLError();
}

GLvoid initVBO()
{
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[0]);
	glBufferData(GL_ARRAY_BUFFER, vertexSize * nvertices, vertices->data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertexbuffers[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * sizeof(GLuint) * ntriangles, faces->data(), GL_STATIC_DRAW);

	delete vertices, faces;
}

GLint main(GLint argc, GLchar **argv)
{
	//Setting up our MVP Matrices
	Model = glm::mat4(1.0f);
	View = glm::lookAt(
		eyePos,
		glm::vec3(0, 0, 0),
		glm::vec3(0, 1, 0));
	Projection = glm::perspective(glm::radians(fov), (GLfloat)wWidth / (GLfloat)wHeight, 0.1f, 100.0f);

	//Read model from .ply file
	p_ply ply = ply_open("turtle.ply", NULL, 0, NULL);
	if (!ply)
		return EXIT_FAILURE;
	if (!ply_read_header(ply))
		return EXIT_FAILURE;
	nvertices = ply_set_read_cb(ply, "vertex", "x", vertex_cb, NULL, 0);
	ply_set_read_cb(ply, "vertex", "y", vertex_cb, NULL, 1);
	ply_set_read_cb(ply, "vertex", "z", vertex_cb, NULL, 2);
	ply_set_read_cb(ply, "vertex", "nx", vertex_cb, NULL, 3);
	ply_set_read_cb(ply, "vertex", "ny", vertex_cb, NULL, 4);
	ply_set_read_cb(ply, "vertex", "nz", vertex_cb, NULL, 5);
	ntriangles = ply_set_read_cb(ply, "face", "vertex_indices", face_cb, NULL, 0);
	if (!ply_read(ply))
		return EXIT_FAILURE;
	ply_close(ply);

	//Read textures from files
	texture_0 = SOIL_load_image("turtle_pan_hrcolor_0.jpg", &wTex, &hTex, &cTex, SOIL_LOAD_RGB);
	texture_90 = SOIL_load_image("turtle_pan_hrcolor_80.jpg", &wTex, &hTex, &cTex, SOIL_LOAD_RGB);
	texture_180 = SOIL_load_image("turtle_pan_hrcolor_180.jpg", &wTex, &hTex, &cTex, SOIL_LOAD_RGB);
	texture_270 = SOIL_load_image("turtle_pan_hrcolor_260.jpg", &wTex, &hTex, &cTex, SOIL_LOAD_RGB);
	normalmap = SOIL_load_image("plasma_normals.jpg", &wNor, &hNor, &cNor, SOIL_LOAD_RGB);
#if defined(__linux__)
	setenv("DISPLAY", ":0", 0);
#endif

	if (false == initGL(&argc, argv))
	{
		return EXIT_FAILURE;
	}

	glGenVertexArrays(1, VertexArrayIDs);
	glGenBuffers(2, vertexbuffers);
	glGenTextures(5, textureArrays);

	computeTangents();
	initShaders();
	initTextures();

	initVBO();

	delete texture_0, texture_90, texture_180, texture_270, normalmap;

	glutMainLoop();

	return EXIT_SUCCESS;
}
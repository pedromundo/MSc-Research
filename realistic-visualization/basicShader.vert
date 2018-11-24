#version 400
in vec3 appPosition_modelspace;
in vec3 appNormal_modelspace;

out vec3 vertPosition_modelspace;
out vec3 vertNormal_modelspace;

uniform mat4 MVP;
uniform mat3 MV;
uniform mat3 M;
uniform vec3 lightPos;
uniform vec3 eyePos;

void main()
{
	vertPosition_modelspace = appPosition_modelspace;
	vertNormal_modelspace = appNormal_modelspace;
	gl_Position = MVP * vec4(vertPosition_modelspace, 1);
}
#version 400
in vec3 appPosition_modelspace;
in vec3 appNormal_modelspace;

out vec3 vertPosition_modelspace;
out vec3 vertNormal_modelspace;
out float vertVertexRotation;

uniform mat4 MVP;
uniform mat3 MV;
uniform mat3 M;
uniform vec3 lightPos;
uniform vec3 eyePos;

void main(){
	//Getting the orientation of the vertex to determine which texture to use
	vec3 front = vec3(0.0,0.0,1.0);
	vec3 vertex_xzdir = normalize(vec3(appPosition_modelspace.x,0.0,appPosition_modelspace.z));
	float vertex_angle = acos(dot(front,vertex_xzdir));

	vertPosition_modelspace = appPosition_modelspace;
	vertNormal_modelspace = appNormal_modelspace;

	if(appPosition_modelspace.x >= 0){
		vertVertexRotation = degrees(vertex_angle);
	}else{
		vertVertexRotation = 360 - degrees(vertex_angle);
	}

	gl_Position = MVP * vec4(vertPosition_modelspace, 1);
}
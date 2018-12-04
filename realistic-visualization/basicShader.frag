#version 400
in vec3 vertPosition_modelspace;
in vec3 vertNormal_modelspace;

//Textures and normal
uniform sampler2D tex_0, tex_90, tex_180, tex_270, nor;
//Lighting parameters
uniform uint lightDiffusePower, lightSpecularPower, lightDistance;
uniform vec3 lightPos, eyePos;
uniform mat4 P_photo;

out vec4 fragColor;

void main()
{
	vec4 LightColor = vec4(1.0, 1.0, 1.0, 1.0);
	vec3 n = normalize(vertNormal_modelspace);
	vec3 l = normalize(lightPos - vertPosition_modelspace);

	float cosTheta = clamp(dot(n, l), 0, 1);

	vec3 E = normalize(eyePos - vertPosition_modelspace);
	vec3 R = reflect(-l, n);
	float cosAlpha = clamp(dot(E, R), 0, 1);

	vec4 MaterialDiffuseColor;
	vec2 uv;

	//Getting the orientation of the vertex to determine which texture to use
	float vertVertexRotation;
	const float PI = 3.1415926535897932384626433832795;
	vec3 front = vec3(0.0, 0.0, 1.0);
	vec3 vertex_xzdir = normalize(vec3(vertPosition_modelspace.x, 0.0, vertPosition_modelspace.z));
	float vertex_angle = acos(dot(front, vertex_xzdir));

	if (vertPosition_modelspace.x > 0)
	{
		vertVertexRotation = (180 * vertex_angle / PI);
	}
	else
	{
		vertVertexRotation = 360 - (180 * vertex_angle / PI);
	}

	vec4 vertPosition_perspective = P_photo * vec4(vertPosition_modelspace,1.0);

	if ((vertVertexRotation >= 0 && vertVertexRotation <= 45) || (vertVertexRotation > 315 && vertVertexRotation <= 360))
	{
		//Front quadrant
		uv = vec2(vertPosition_perspective.x / 3.000 + 0.470,-vertPosition_perspective.y / 4.000 + 0.530);
		MaterialDiffuseColor = texture(tex_0, uv);
	}
	else if (vertVertexRotation > 45 && vertVertexRotation <= 135)
	{
		//Left-side quadrant
		uv = vec2(vertPosition_perspective.z / 2.000 + 0.590,-vertPosition_perspective.y / 4.000 + 0.535);
		MaterialDiffuseColor = texture(tex_90, uv);
	}
	else if (vertVertexRotation > 135 && vertVertexRotation <= 225)
	{
		//Back quadrant
		uv = vec2(-vertPosition_perspective.x / 3.500 + 0.490,-vertPosition_perspective.y / 4.000 + 0.530);
		MaterialDiffuseColor = texture(tex_180, uv);
	}
	else if (vertVertexRotation > 225 && vertVertexRotation <= 315)
	{
		//Right-side quadrant
		uv = vec2(-vertPosition_perspective.z / 2.000 + 0.350,-vertPosition_perspective.y / 4.000 + 0.525);
		MaterialDiffuseColor = texture(tex_270, uv);
	}

	//Feathering texture boundaries
	if (vertVertexRotation > 43 && vertVertexRotation < 45){
		vec2 uv2 = vec2(vertPosition_perspective.z / 2.000 + 0.590,-vertPosition_perspective.y / 4.000 + 0.535);
		MaterialDiffuseColor = mix(texture(tex_0, uv),texture(tex_90, uv2),(vertVertexRotation-43)/2);
	}
	else if (vertVertexRotation > 133 && vertVertexRotation < 135){
		vec2 uv2 = vec2(-vertPosition_perspective.x / 3.500 + 0.490,-vertPosition_perspective.y / 4.000 + 0.530);
		MaterialDiffuseColor = mix(texture(tex_90, uv),texture(tex_180, uv2),(vertVertexRotation-133)/2);
	}
	else if (vertVertexRotation > 223 && vertVertexRotation < 225){
		vec2 uv2 = vec2(-vertPosition_perspective.z / 2.000 + 0.350,-vertPosition_perspective.y / 4.000 + 0.525);
		MaterialDiffuseColor = mix(texture(tex_180, uv),texture(tex_270, uv2),(vertVertexRotation-223)/2);
	}
	else if (vertVertexRotation > 313 && vertVertexRotation < 315){
		vec2 uv2 = vec2(vertPosition_perspective.x / 3.000 + 0.470,-vertPosition_perspective.y / 4.000 + 0.530);
		MaterialDiffuseColor = mix(texture(tex_270, uv),texture(tex_0, uv2),(vertVertexRotation-313)/2);
	}

	vec4 MaterialAmbientColor = vec4(0.2, 0.2, 0.2, 1.0) * MaterialDiffuseColor;
	vec4 MaterialSpecularColor = vec4(0.663, 0.553, 0.467, 1.0);

	vec4 color = MaterialAmbientColor +
				 MaterialDiffuseColor * LightColor * lightDiffusePower * cosTheta / (lightDistance * lightDistance) +
				 MaterialSpecularColor * LightColor * lightSpecularPower * pow(cosAlpha, 30) / (lightDistance * lightDistance);
	fragColor = color;
}
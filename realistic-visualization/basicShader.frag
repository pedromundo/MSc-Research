#version 400
in vec3 vertPosition_modelspace;
in vec3 vertNormal_modelspace;
in float vertVertexRotation;

//Textures and normal
uniform sampler2D tex_0, tex_90, tex_180, tex_270, nor;
//Lighting parameters
uniform uint lightDiffusePower, lightSpecularPower, lightDistance;
uniform vec3 lightPos, eyePos;

out vec4 fragColor;

void main(){
	vec4 LightColor = vec4(1.0, 1.0, 1.0, 1.0);
	vec3 n = normalize(vertNormal_modelspace);
	vec3 l = normalize(lightPos - vertPosition_modelspace);

	float cosTheta = clamp(dot(n, l), 0, 1);

	vec3 E = normalize(eyePos - vertPosition_modelspace);
	vec3 R = reflect(-l, n);
	float cosAlpha = clamp(dot(E, R), 0, 1);

	vec4 MaterialDiffuseColor;
	vec2 uv;

	if ((vertVertexRotation >= 0 && vertVertexRotation <= 45) || (vertVertexRotation >= 315 && vertVertexRotation <= 360)){
		//Quadrante frontal
		uv.x = vertPosition_modelspace.x/3.000+0.475;
		uv.y = -vertPosition_modelspace.y/2.250+0.535;
		MaterialDiffuseColor = texture(tex_0, uv);
	}else if(vertVertexRotation > 45 && vertVertexRotation <= 135){
		//Quadrante lateral esquerdo
		uv.x = vertPosition_modelspace.z/3.000+0.500;
		uv.y = -vertPosition_modelspace.y/2.250+0.528;
		MaterialDiffuseColor = texture(tex_90, uv);
	}else if(vertVertexRotation >= 135 && vertVertexRotation <= 225){
		//Quadrante traseiro
		uv.x = -vertPosition_modelspace.x/3.000+0.475;
		uv.y = -vertPosition_modelspace.y/2.250+0.535;
		MaterialDiffuseColor = texture(tex_180, uv);
	}else if(vertVertexRotation > 225 && vertVertexRotation < 315){
		//Quadrante lateral direito
		uv.x = -vertPosition_modelspace.z/3.000+0.500;
		uv.y = -vertPosition_modelspace.y/2.250+0.550;
		MaterialDiffuseColor = texture(tex_270, uv);
	}

	//MaterialDiffuseColor = vec4(1.0);

	//Interpolating the rotation (in Y) between 0..1
	//MaterialDiffuseColor = vec4(vec3(vertVertexRotation/360),1);
	vec4 MaterialAmbientColor = vec4(0.2, 0.2, 0.2, 1.0) * MaterialDiffuseColor;
	vec4 MaterialSpecularColor = vec4(1.0, 1.0, 1.0, 1.0);

	vec4 color = MaterialAmbientColor +
				 MaterialDiffuseColor * LightColor * lightDiffusePower * cosTheta / (lightDistance * lightDistance) +
				 MaterialSpecularColor * LightColor * lightSpecularPower * pow(cosAlpha, 10) / (lightDistance * lightDistance);
	fragColor = color;
}
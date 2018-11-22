#version 400
in vec2 vertTexCoord;
in vec3 vertLightDirection_tangentspace;
in vec3 vertEyeDirection_tangentspace;
in vec3 vertPosition_modelspace;
in vec3 vertNormal_tangentspace;
in float vertVertexRotation;

//Texture and normal
uniform sampler2D tex, tex_back, nor;
uniform uint lightDiffusePower, lightSpecularPower, lightDistance;

out vec4 fragColor;

void main() {
	vec4 LightColor = vec4(1.0,1.0,1.0,1.0);
	vec3 n = normalize( vertNormal_tangentspace );
	vec3 l = normalize( vertLightDirection_tangentspace );

	float cosTheta = clamp( dot( n,l ), 0,1 );

	vec3 E = -normalize(vertEyeDirection_tangentspace);
	vec3 R = reflect(-l,n);
	float cosAlpha = clamp( dot( E,R ), 0,1 );

	vec4 MaterialDiffuseColor = texture(tex,vertTexCoord);
	//Interpolating the rotation (in Y) between 0..1
	// vec4 MaterialDiffuseColor = vec4(vec3(vertVertexRotation/360),1);
	if(vertPosition_modelspace.z < 0)
		MaterialDiffuseColor = texture(tex_back,vertTexCoord);
	vec4 MaterialAmbientColor = vec4(0.2,0.2,0.2,1.0) * MaterialDiffuseColor;
	vec4 MaterialSpecularColor = vec4(1.0,1.0,1.0,1.0);

    vec4 color = MaterialAmbientColor +
	MaterialDiffuseColor * LightColor * lightDiffusePower * cosTheta / (lightDistance*lightDistance) +
    MaterialSpecularColor * LightColor * lightSpecularPower * pow(cosAlpha,10) / (lightDistance*lightDistance);
	fragColor = MaterialDiffuseColor;
}
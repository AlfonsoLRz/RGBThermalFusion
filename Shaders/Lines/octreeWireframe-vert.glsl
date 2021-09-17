#version 450

layout (location = 0) in vec4 vPosition;

// Instancing-related attributes
layout (location = 10) in vec3 vTranslation;
layout (location = 12) in vec3 vScale;

uniform mat4 mModelViewProj;			

void main() {
	vec3 scale = 1.0f / vScale;
	
	gl_Position = mModelViewProj * vec4(vPosition.xyz / scale + vTranslation, 1.0f);
}
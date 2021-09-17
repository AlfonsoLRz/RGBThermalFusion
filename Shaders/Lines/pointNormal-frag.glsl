#version 450

subroutine vec3 colorType();
subroutine uniform colorType colorUniform;

in vec3 color;

uniform vec3 uniformNormalColor;

layout (location = 0) out vec4 fColor;

subroutine(colorType)
vec3 normalColor()
{
	return (normalize(color) + 1.0f) / 2.0f;
}

subroutine(colorType)
vec3 uniformColor()
{
	return uniformNormalColor;
}

void main() {
	fColor = vec4(colorUniform(), 1.0f);
}
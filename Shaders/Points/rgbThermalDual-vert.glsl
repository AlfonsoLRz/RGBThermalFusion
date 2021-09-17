#version 450

layout (location = 0) in vec4 vPosition;
layout (location = 4) in vec3 vColor;
layout (location = 8) in vec4 vSecondaryColor04;

subroutine void outlierType(vec3 rgbColor, vec3 thermalColor, float temperature, float absoluteMeanTemperature);
subroutine uniform outlierType outlierUniform;

uniform vec2		boundaries;
uniform float		gammaRGB;
uniform float		interquartileRange;
uniform float		meanTemperature;
uniform mat4		mModelViewProj;
uniform float		pointSize;	
uniform vec2		q1q3;
uniform sampler2D	scaleTexture;

out vec4 color;

subroutine(outlierType)
void outlierColor(vec3 rgbColor, vec3 thermalColor, float temperature, float absoluteMeanTemperature)
{
	float normalizedQ1 = clamp((q1q3.x - interquartileRange - boundaries.x) / (boundaries.y - boundaries.x), .0f, 1.0f);
	float normalizedQ3 = clamp((q1q3.y + interquartileRange - boundaries.x) / (boundaries.y - boundaries.x), .0f, 1.0f);
	float wt;

	if (temperature < absoluteMeanTemperature)
	{
		wt = smoothstep(.0f, 1.0f, (absoluteMeanTemperature - temperature) / (absoluteMeanTemperature - boundaries.x));
	}
	else
	{
		wt = smoothstep(.0f, 1.0f, (temperature - absoluteMeanTemperature) / (boundaries.y - absoluteMeanTemperature));
	}

	if (temperature < q1q3.x - interquartileRange || temperature > q1q3.y + interquartileRange)
	{
		wt = 1.0f;
	}

	color = vec4(wt * thermalColor + gammaRGB * (1.0f - wt) * rgbColor, vSecondaryColor04.w);

}

subroutine(outlierType)
void noOutlierColor(vec3 rgbColor, vec3 thermalColor, float temperature, float absoluteMeanTemperature) 
{
	float wt;

	if (temperature < absoluteMeanTemperature)
	{
		wt = (absoluteMeanTemperature - temperature) / (absoluteMeanTemperature - boundaries.x);
	}
	else
	{
		wt = (temperature - absoluteMeanTemperature) / (boundaries.y - absoluteMeanTemperature);
	}

	color = vec4(wt * thermalColor + gammaRGB * (1.0f - wt) * rgbColor, vSecondaryColor04.w);
}


void main() {
	float temperature = vSecondaryColor04.y, absoluteMeanTemperature = meanTemperature * (boundaries.y - boundaries.x) + boundaries.x, wt;
	vec3 rgbColor = vec3(vColor.x);
	vec3 thermalColor = texture(scaleTexture, vec2(.5f, (temperature - boundaries.x) / (boundaries.y - boundaries.x))).xyz;

	outlierUniform(rgbColor, thermalColor, temperature, absoluteMeanTemperature);

	gl_PointSize = pointSize;
	gl_Position = mModelViewProj * vPosition;
}
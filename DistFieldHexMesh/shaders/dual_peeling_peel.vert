//--------------------------------------------------------------------------------------
// Order Independent Transparency with Dual Depth Peeling
//
// Author: Louis Bavoil
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

uniform UniformBufferObject {
	mat4 modelView;
	mat4 proj;
	vec4 defColor;
	float ambient;
  int useDefColor;
  int normalShadingOn;
  int twoSideLighting;
  int numLights;
	vec3 lightDir[8];
};

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec3 fragNormal;

void main(void)
{
  gl_Position = proj * modelView * vec4(inPosition, 1.0);
  vec3 blackColor = vec3(0.0, 0.0, 0.0);
	
	if (useDefColor != 0)
		fragColor = defColor;
	else
		fragColor = vec4(inColor, 1);

  fragNormal = normalize((modelView * vec4(inNormal, 0.0)).xyz);
}

//--------------------------------------------------------------------------------------
// Order Independent Transparency with Dual Depth Peeling
//
// Author: Louis Bavoil
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension ARB_draw_buffers : require

layout(location = 0) uniform sampler2DRect frontBlenderSampler;
layout(location = 1) uniform sampler2DRect backBlenderSampler;

layout(location = 0) out vec4 outColor; // Color is showing up. This binding is valid

void main(void)
{
	vec4 frontColor = texture2DRect(frontBlenderSampler, gl_FragCoord.xy);
	vec3 backColor = texture2DRect(backBlenderSampler, gl_FragCoord.xy).rgb;
  float alpha = frontColor.w;
  alpha = 0.95;
	float alphaComp = 1.0 - alpha;

	// front + back
	outColor = vec4(frontColor.rgb * alpha + backColor * alphaComp, 1);

#if 1
  float r = gl_FragCoord.x / 784.0;
  float g = gl_FragCoord.y / 918.0;
  
  float k = 0.75;
  vec3 blendedColor = k * vec3(r, g, 0) + (1-k) * outColor.rgb;
  outColor = vec4(blendedColor, 1);

	// front blender
	//outColor.rgb = frontColor + vec3(alphaComp);
	
	// back blender
	//outColor.rgb = backColor;
#endif
}

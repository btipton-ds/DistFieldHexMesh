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

layout(location = 0) uniform sampler2DRect backColorSampler;

layout(location = 0) out vec4 outColor;

void main(void)
{
	outColor = texture(backColorSampler, gl_FragCoord.xy);
  
  float r = gl_FragCoord.x / 784.0;
  float g = gl_FragCoord.y / 918.0;
  
  float k = 0.75;
  vec3 blendedColor = k * vec3(r, g, 0) + (1-k) * outColor.rgb;
  outColor = vec4(blendedColor, 1);

	// for occlusion query
//	if (outColor.a == 0) 
//    discard;
}

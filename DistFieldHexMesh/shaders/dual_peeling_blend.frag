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

layout(location = 0) uniform sampler2DRect backColorSampler;

layout(location = 0) out vec4 outColor;

void main(void)
{
	outColor = texture(backColorSampler, gl_FragCoord.xy);
  
	// for occlusion query
	if (outColor.a == 0) 
    discard;
}

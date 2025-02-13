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

uniform sampler2D source;

layout(location = 0) out vec4 outColor;

void main(void)
{
	outColor = texture(source, gl_FragCoord.xy);
//  outColor = vec4(0,1,0,1);

	// for occlusion query
	if (outColor.a == 0) 
    discard;
}

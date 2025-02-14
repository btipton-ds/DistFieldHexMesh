//--------------------------------------------------------------------------------------
// Order Independent Transparency with Dual Depth Peeling
//
// Author: Louis Bavoil
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

uniform sampler2D tempSampler;

void main(void)
{
	gl_FragColor = texture(tempSampler, gl_FragCoord.xy);

	// for occlusion query
	if (gl_FragColor.a == 0) 
    discard;
}

//--------------------------------------------------------------------------------------
// Order Independent Transparency with Dual Depth Peeling
//
// Author: Louis Bavoil
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

uniform sampler2DRect frontBlenderSampler;
uniform sampler2DRect backBlenderSampler;

void main(void)
{
	vec4 frontColor = texture(frontBlenderSampler, gl_FragCoord.xy);
	vec3 backColor = texture(backBlenderSampler, gl_FragCoord.xy).rgb;
	float alphaMultiplier = 1.0 - frontColor.w;

	// front + back
	gl_FragColor.rgb = frontColor + backColor * alphaMultiplier;
	
	// front blender
	//gl_FragColor.rgb = frontColor + vec3(alphaMultiplier);
	
	// back blender
	//gl_FragColor.rgb = backColor;
}

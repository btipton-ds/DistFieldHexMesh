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

// This is difficult to find. sampler2D uses 0-1 range. Sampler2DRect uses 0-width and 0-height, NOT normalized
// The vertex shader passes xy in screen coordinates to match
uniform sampler2DRect source; 

layout(location = 0) out vec4 outColor;

void main(void)
{
	outColor = texture(source, gl_FragCoord.xy);
  #if 0
  if (gl_FragCoord.x < 0)
    outColor = vec4(1,0,0,1);
  else if(gl_FragCoord.x < 392)
    outColor = vec4(0,1,0,1);
  else
    outColor = vec4(0,0,1,1);
#endif
}

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

layout(location = 0) in vec3 inPosition;

void main(void)
{
// The passes xy in screen coordinates to match use of Sampler2DRect in the fragment shader
// This is difficult to find. sampler2D uses 0-1 range. Sampler2DRect uses 0-width and 0-height, NOT normalized.
  gl_Position = vec4(inPosition, 1.0);
}

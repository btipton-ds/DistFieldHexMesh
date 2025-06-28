/*

This file is part of the VulkanQuickStart Project.

	The VulkanQuickStart Project is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VulkanQuickStart Project is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the TriMesh Library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the VulkanQuickStart Project (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/

  Derived from Dual Depth Peeling by
  Author: Louis Bavoil
  Email: sdkfeedback@nvidia.com

  Copyright (c) NVIDIA Corporation. All rights reserved.
*/

#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension ARB_draw_buffers : require

layout(location = 0) uniform sampler2DRect depthBlenderSampler;
layout(location = 1) uniform sampler2DRect frontBlenderSampler;

_COMMON_UBOS_

layout(location = 0) in vec4 fragColor;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec3 echoInPosition;

layout(location = 0) out vec4 out_0;
layout(location = 1) out vec4 out_1;
layout(location = 2) out vec4 out_2;

#define MAX_DEPTH 1.0

vec4 shadeFragment()
{
  float C_PI = radians(180);
  float intensity = 0.0;
  vec4 color;

  if (normalShadingOn != 0) {
    for (int i = 0; i < numLights; i++) {
		float dp;
		switch (i) {
			default:
			case 0:
				dp = dot(lightDir0.xyz, fragNormal);
				break;
			case 1:
				dp = dot(lightDir1.xyz, fragNormal);
				break;
			case 2:
				dp = dot(lightDir2.xyz, fragNormal);
				break;
			case 3:
				dp = dot(lightDir3.xyz, fragNormal);
				break;
			case 4:
				dp = dot(lightDir4.xyz, fragNormal);
				break;
			case 5:
				dp = dot(lightDir5.xyz, fragNormal);
				break;
			case 6:
				dp = dot(lightDir6.xyz, fragNormal);
				break;
			case 7:
				dp = dot(lightDir7.xyz, fragNormal);
				break;
		}
		if (twoSideLighting != 0)
			dp = abs(dp);

		if (dp > 0)
			intensity += dp;
    }
	intensity /= numLights;
  } else {
    intensity = 1.0;
  }
  
  intensity = min(intensity, 1.0);

  intensity = ambient + (1.0 - ambient) * intensity;

  float alpha = fragColor[3];

  color = intensity * fragColor;

  color[3] = alpha;

  return color;
}

void main() {
	if (clippingPlaneOn == 1) {
		vec3 v = echoInPosition - clippingPlaneOrigin.xyz;
		float dp = dot(v, clippingPlaneNormal.xyz);
		if (dp < 0)
			discard;
	}


	// window-space depth interpolated linearly in screen space
	float fragDepth = gl_FragCoord.z;
	vec2 depthBlender = texture2DRect(depthBlenderSampler, gl_FragCoord.xy).xy;
	vec4 forwardTemp = texture2DRect(frontBlenderSampler, gl_FragCoord.xy);

	// Depths and 1.0-alphaMult always increase
	// so we can use pass-through by default with MAX blending
	out_0.xy = depthBlender;
	
	// Front colors always increase (DST += SRC*ALPHA_MULT)
	// so we can use pass-through by default with MAX blending
	out_1 = forwardTemp;

	// Because over blending makes color increase or decrease,
	// we cannot pass-through by default.
	// Each pass, only one fragment writes a color greater than 0
	out_2 = vec4(0.0); // I can't find this in the spec, but it looks like a single value is duplicated for all entries

	float nearestDepth = -depthBlender.x;
	float farthestDepth = depthBlender.y;
	float alphaMultiplier = 1.0 - forwardTemp.w;

	if (fragDepth < nearestDepth || fragDepth > farthestDepth) {
		// Skip this depth in the peeling algorithm
		out_0.xy = vec2(-MAX_DEPTH);
		return;
	}
	
	if (fragDepth > nearestDepth && fragDepth < farthestDepth) {
		// This fragment needs to be peeled again
		out_0.xy = vec2(-fragDepth, fragDepth);
		return;
	}
	
	// If we made it here, this fragment is on the peeled layer from last pass
	// therefore, we need to shade it, and make sure it is not peeled any farther
	vec4 color = shadeFragment();
	out_0.xy = vec2(-MAX_DEPTH);
	
	if (fragDepth == nearestDepth) {
		out_1.xyz += color.rgb * color.a * alphaMultiplier;
		out_1.w = 1.0 - alphaMultiplier * (1.0 - color.a);
	} else {
		out_2 += color;
	}
}

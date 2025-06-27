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

_COMMON_UBOS_

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec3 fragNormal;

void main() {
	int ignoreClipped = 0;
	if (clippingPlaneOn == 1) {
		vec3 v = inPosition - clippingPlaneOrigin.xyz;
		float dp = dot(clippingPlaneNormal.xyz, v);
		ignoreClipped = dp < 0 ? 1 : 0;
	}

	if (ignoreClipped == 1) {
		fragColor = vec4(0,0,0,0);
	} else {
		gl_Position = proj * modelView * vec4(inPosition, 1.0);
		fragNormal = normalize((modelView * vec4(inNormal, 0.0)).xyz);
	
		if (useDefColor != 0)
			fragColor = defColor;
		else
			fragColor = vec4(inColor, 1);
	}
}


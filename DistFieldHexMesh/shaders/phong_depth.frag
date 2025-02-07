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

*/

#version 450
#extension GL_ARB_separate_shader_objects : enable

#define MAX_DEPTH 99999.0

precision highp float;
precision highp sampler2D;

uniform UniformBufferObject {
	mat4 modelView;
	mat4 proj;
	vec4 defColor;
	float ambient;
  int useDefColor;
  int normalShadingOn;
  int twoSideLighting;
  int numLights;
	vec3 lightDir[8];
};

layout(location = 0) in vec4 fragColor;
layout(location = 1) in vec3 fragNormal;

uniform sampler2D uDepth;
uniform sampler2D uFrontColor;

layout (location = 0) out vec2 depth;  // RG32F, R - negative front depth, G - back depth
layout (location = 1) out vec4 frontColor;
layout (location = 2) out vec4 backColor;
layout (location = 3) out vec4 outColor;

void main() {
  float fragDepth = gl_FragCoord.z;   // 0 - 1

  ivec2 fragCoord = ivec2(gl_FragCoord.xy);
  vec2 lastDepth = texelFetch(uDepth, fragCoord, 0).rg;
  vec4 lastFrontColor = texelFetch(uFrontColor, fragCoord, 0);

  // depth value always increases
  // so we can use MAX blend equation
  depth.rg = vec2(-MAX_DEPTH);

  // front color always increases
  // so we can use MAX blend equation
  frontColor = lastFrontColor;

  // back color is separately blend afterwards each pass
  backColor = vec4(0.0);

  float nearestDepth = - lastDepth.x;
  float furthestDepth = lastDepth.y;
  float alphaMultiplier = 1.0 - lastFrontColor.a;

  if (fragDepth < nearestDepth || fragDepth > furthestDepth) {
      // Skip this depth since it's been peeled.
      return;
  }

  if (fragDepth > nearestDepth && fragDepth < furthestDepth) {
      // This needs to be peeled.
      // The ones remaining after MAX blended for 
      // all need-to-peel will be peeled next pass.
      depth.rg = vec2(-fragDepth, fragDepth);
      return;
  }

  float C_PI = radians(180);
  float intensity = 0.0;

  if (normalShadingOn != 0) {
    for (int i = 0; i < numLights; i++) {
      float dp = dot(lightDir[i], fragNormal);

      if (twoSideLighting != 0)
        dp = abs(dp);

      if (dp > 0)
        intensity += dp;
    }
  } else {
    intensity = 1.0;
  }
  
  intensity = min(intensity, 1.0);

  intensity = ambient + (1.0 - ambient) * intensity;

  float alpha = fragColor[3];

  vec4 color = vec4(intensity * fragColor.rgb, alpha);

  outColor = color;

  if (fragDepth == nearestDepth) {
      frontColor.rgb += color.rgb * color.a * alphaMultiplier;
      frontColor.a = 1.0 - alphaMultiplier * (1.0 - color.a);
  } else {
      backColor += color;
  }
}

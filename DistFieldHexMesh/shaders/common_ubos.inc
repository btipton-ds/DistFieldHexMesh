uniform UniformBufferObject {
	mat4 modelView;
	mat4 proj;
	vec4 defColor;
	float ambient;
	int useDefColor;
	int normalShadingOn;
	int twoSideLighting;
	int numLights;
	vec3 lightDir[4];
};

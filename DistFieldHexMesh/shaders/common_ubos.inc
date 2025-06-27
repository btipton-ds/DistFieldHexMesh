uniform UniformBufferObject {
	mat4 modelView;
	mat4 proj;
	vec4 defColor;
	vec4 lightDir0;
	vec4 lightDir1;
	float ambient;
	int useDefColor;
	int normalShadingOn;
	int twoSideLighting;
	int numLights;
};

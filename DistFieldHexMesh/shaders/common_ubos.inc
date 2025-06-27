uniform UniformBufferObject {
	mat4 modelView;
	mat4 proj;
	vec4 defColor;
	vec4 lightDir0;
	vec4 lightDir1;
	vec4 lightDir2;
	vec4 lightDir3;
	vec4 lightDir4;
	vec4 lightDir5;
	vec4 lightDir6;
	vec4 lightDir7;
	float ambient;
	int useDefColor;
	int normalShadingOn;
	int twoSideLighting;
	int numLights;
};

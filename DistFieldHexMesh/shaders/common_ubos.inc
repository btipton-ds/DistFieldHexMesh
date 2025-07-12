layout (shared) uniform UniformBufferObject {
	mat4 modelView;
	mat4 proj;
	vec4 defColor;
	vec4 backColor;
	vec4 lightDir0;
	vec4 lightDir1;
	vec4 lightDir2;
	vec4 lightDir3;
	vec4 lightDir4;
	vec4 lightDir5;
	vec4 lightDir6;
	vec4 lightDir7;

	vec4 clippingPlane0Origin;
	vec4 clippingPlane0Normal;

	vec4 clippingPlane1Origin;
	vec4 clippingPlane1Normal;

	float ambient;
	int useDefColor;
	int useBackColor;
	int normalShadingOn;
	int twoSideLighting;
	int numLights;

	int clippingPlane0On;
	int clippingPlane1On;
};

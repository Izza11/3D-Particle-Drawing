#version 400        

subroutine void RenderPassType();
subroutine uniform RenderPassType RenderPass;

uniform float time;
uniform mat4 PVM;
uniform mat4 rotatePoints;
uniform vec3 handTranslation;

layout (location = 0) in vec3 curpos;
layout (location = 1) in vec3 initpos;

out vec3 outPosition;
out vec3 initOutPos;

subroutine (RenderPassType)
	void update() {
		
		vec3 translateback = curpos - initpos; // translated back to the origin
		vec3 a = curpos - translateback;
		vec4 b = rotatePoints * vec4(a, 1.0);        // rotate at origin
		vec3 c = vec3(b) + handTranslation;

		outPosition = c;
		initOutPos = initpos;

	}

subroutine (RenderPassType)
	void render() {
		gl_Position = PVM * vec4(curpos, 1.0);
	}


void main(void)
{
	// This will call either render() or update()
	 RenderPass();
	
}
#version 400        

subroutine void RenderPassType();
subroutine uniform RenderPassType RenderPass;

uniform float time;
uniform mat4 PVM;

layout (location = 0) in vec3 curpos;

out vec3 outPosition;


subroutine (RenderPassType)
	void update() {
		outPosition = curpos;
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
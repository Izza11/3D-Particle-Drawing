#version 400        

subroutine void RenderPassType();
subroutine uniform RenderPassType RenderPass;

uniform float time;
uniform mat4 PVM;
uniform mat4 rotatePoints;
uniform vec3 handTranslation;
uniform vec3 Pcolor;

layout (location = 0) in vec3 curpos;
layout (location = 1) in float partTime;
layout (location = 2) in float partPlaced;
layout (location = 3) in float partFade;

out vec3 outPosition;
out float partOutTime;
out float partOutPlaced;
out float partOutFade;
out float alpha;


subroutine (RenderPassType)
	void update() {

		if (partTime < time && partPlaced == 0.0) {                 // partplaced == false, particle hasnt been placed
			outPosition = handTranslation;
			partOutPlaced = 1.0;
			partOutFade = partFade;
			partOutTime = partTime;
		} else {
			outPosition = curpos;	
			partOutPlaced = partPlaced;	
			partOutFade = partFade;
			partOutTime = partTime;
		}
		if (partPlaced == 1.0) {
			partOutFade = partFade - 0.001;
			partOutTime = partTime;
			outPosition = curpos;	
		}

		if(partFade <= 0.0) {
			partOutPlaced = 0.0;
			partOutTime = partTime + time;
			partOutFade = 1.0;
			outPosition = vec3(0.0, -200.0, 0.0);	
		}
		
	}

subroutine (RenderPassType)
	void render() {
		gl_Position = PVM * vec4(curpos, 1.0);
		alpha = partFade;
	}

void main(void)
{
	// This will call either render() or update()
	 RenderPass();
	
}
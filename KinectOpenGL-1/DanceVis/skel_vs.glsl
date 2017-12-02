#version 450        

in vec3 pos_attrib;

uniform mat4 PVM;



void main(void)
{
	gl_Position = PVM*vec4(pos_attrib, 1.0);
}
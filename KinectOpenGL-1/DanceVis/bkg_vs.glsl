#version 450        

layout(location = 0) in vec3 pos_attrib;

out vec2 tex_coord;

void main(void)
{
	gl_Position = vec4(pos_attrib, 1.0);
	tex_coord = pos_attrib.xy*0.5 + vec2(0.5);
}
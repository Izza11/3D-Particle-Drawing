#version 450        


layout (points) in;
layout (triangle_strip, max_vertices = 8) out;
uniform mat4 PVM;

void main(void) {    

	vec4 offset = vec4(-0.2, 0.2, 0.0, 0.0);
	vec4 vertexPos = offset + gl_in[0].gl_Position;
    gl_Position = PVM * vertexPos;
    EmitVertex();

	offset = vec4(-0.2, -0.2, 0.0, 0.0);
	vertexPos = offset + gl_in[0].gl_Position;
    gl_Position = PVM * vertexPos;
    EmitVertex();

	offset = vec4(0.2, 0.2, 0.0, 0.0);
	vertexPos = offset + gl_in[0].gl_Position;
    gl_Position = PVM * vertexPos;
    EmitVertex();

	offset = vec4(0.2, -0.2, 0.0, 0.0);
	vertexPos = offset + gl_in[0].gl_Position;
    gl_Position = PVM * vertexPos;
    EmitVertex();
    EndPrimitive();
}  
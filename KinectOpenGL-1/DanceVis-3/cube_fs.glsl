#version 400

uniform samplerCube cubemap;

out vec4 fragcolor;           
in vec3 pos;
     
void main(void)
{   
   //fragcolor = vec4(pos, 1.0);
   fragcolor = texture(cubemap, pos);
}





















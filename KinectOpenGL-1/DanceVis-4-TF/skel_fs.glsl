#version 400  

out vec4 fragcolor;
in vec3 outPosition;
in float alpha;
uniform vec3 Pcolor;
void main(void)
{   
   fragcolor = vec4(Pcolor, alpha);  

}





















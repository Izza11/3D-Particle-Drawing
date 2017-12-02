#version 450
#extension GL_EXT_gpu_shader4 : enable

out vec4 fragcolor;   

layout(location = 0) uniform float time;
layout(location = 1) uniform sampler2D colorTex0;
layout(location = 2) uniform sampler2D colorTex1;
layout(location = 3) uniform sampler2D colorTex2;
layout(location = 4) uniform sampler2D colorTex3;

layout(location = 5) uniform int background_mode;
layout(location = 6) uniform float params[10];


in vec2 tex_coord;

#define PI 3.14159268
#define LAYERS 4.


float wave(float a, vec2 p) {
	return (cos(dot(p,vec2(cos(a),sin(a)))) + 1.) / 2.;
}


float wrap(float v) {
    float f = fract(v);
    return v-f < 2e-4 ? f : 1. - f;
}


float quasicrystal(vec2 p) {
    float sum = 0.;
    for(float a = 0.; a < PI; a += PI/LAYERS) 
        sum += wave(a, p);

    return wrap(sum);
}


void main(void)
{   
    if(background_mode == 0)
	{
		fragcolor = texture(colorTex0, tex_coord);
		return;
	}
	else if(background_mode == 1)
	{
		fragcolor = texture(colorTex1, tex_coord);
		return;
	}
	else if(background_mode == 2)
	{
		fragcolor = texture(colorTex2, tex_coord);
		return;
	}
	else if(background_mode == 3)
	{
		fragcolor = texture(colorTex3, tex_coord);
		return;
	}
	else if(background_mode == 4)
	{
		fragcolor = texture(colorTex0, tex_coord)*texture(colorTex1, tex_coord+vec2(time, time*params[9]));
		return;
	}
	else if(background_mode == 5)
	{
		float a = quasicrystal(params[0]*200.0*tex_coord);
		fragcolor = mix(texture(colorTex1, tex_coord), texture(colorTex2, tex_coord), a+sin(time));
		return;		
	}
	else if(background_mode == 6)
	{

		vec2 res = vec2(1920.0, 1080.0);
		// normalized coordinates (-1 to 1 vertically)
		vec2 p = (-res.xy + 2.0*gl_FragCoord.xy)/res.y;

		// angle of each pixel to the center of the screen
		float a = atan(p.y,p.x);
    
		//float r = pow( pow(p.x*p.x,4.0) + pow(p.y*p.y,4.0), 1.0/8.0 );
		 float r = max( abs(p.x), abs(p.y) );
    

		vec2 uv = vec2( 0.3/r + 0.2*time, a/3.1415927 );

		vec2 uv2 = vec2( uv.x, atan(p.y,abs(p.x))/3.1415927 );
		vec3 col =  texture2DGrad( colorTex2, uv, dFdx(uv2), dFdy(uv2) ).xyz;
    
		// darken at the center    
		col = col*r;
    
		fragcolor = vec4( col, 1.0 );
		return;		
	}

}





















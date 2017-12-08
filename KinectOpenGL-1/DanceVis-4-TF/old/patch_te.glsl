#version 430 core 
layout (quads, equal_spacing, ccw) in;

layout(location = 1) uniform float time = 0.0;

layout(location = 4) uniform float params[10];

struct pnQuad
{
	/*
	vec3 b0, b1, b2, b3;
	
	vec3 b01, b02, b03;
	vec3 b10, b12, b13;
	vec3 b20, b21, b23;
	vec3 b30, b31, b32;
	*/

	vec3 b00, b01, b02, b03;
	vec3 b10, b11, b12, b13;
	vec3 b20, b21, b22, b23;
	vec3 b30, b31, b32, b33;

	/*
	vec3 n0, n1, n2, n3;
	vec3 n01, n12, n23, n30;
	vec3 n0123;
	*/
	vec3 n00, n01, n02;
	vec3 n10, n11, n12;
	vec3 n20, n21, n22;
};

patch in pnQuad pn;

out vec3 gnormal;
out vec3 gpos;
out vec2 gtex_coord;

in float frame_id_te[];
out float frame_id_g;

vec3 interpolate3(in vec3 v0, in vec3 v1, in vec3 v2, in vec3 v3)
{
	vec3 a = mix(v0, v1, gl_TessCoord.x);
	vec3 b = mix(v3, v2, gl_TessCoord.x);
	return mix(a, b, gl_TessCoord.y);
}

float interpolate1(in float v0, in float v1, in float v2, in float v3)
{
	float a = mix(v0, v1, gl_TessCoord.x);
	float b = mix(v3, v2, gl_TessCoord.x);
	return mix(a, b, gl_TessCoord.y);
}

vec3 disp(vec3 pos, vec3 a) 
{
   float x = sin(a.x*pos.x)*sin(a.x*pos.y)*sin(a.x*pos.z);
   float y = sin(a.y*pos.x)*sin(a.y*pos.y)*sin(a.y*pos.z);
   float z = sin(a.z*pos.x)*sin(a.z*pos.y)*sin(a.z*pos.z);

   return 1.5*vec3(x,y,z);
}

vec4 B(float t)
{
   float a = 1.0-t;
   return vec4(a*a*a, 3.0*a*a*t, 3.0*a*t*t, t*t*t);
}

vec4 Bn(float t)
{
   float a = 1.0-t;
   return vec4(a*a, 2.0*a*t, t*t, 0.0);
}

vec4 aa_pn_quads_basis (float x)
{
  return vec4 (1.0, x, x * x, x * x * x);
}


void main()
{
	gtex_coord = gl_TessCoord.xy;

	float u = gl_TessCoord.x;
	float v = gl_TessCoord.y;
 
	// compute linear interp position
	//vec3 pnPos  = interpolate3(pn.b0, pn.b1, pn.b2, pn.b3);
	//vec3 pnNorm = normalize(interpolate3(pn.n0, pn.n1, pn.n2, pn.n3));


	/*
	// compute PN position
	vec4 Bu = B(u);
	vec4 Bv = B(v);

	pnPos =	Bu[0]*Bv[0]*pn.b0  + Bu[1]*Bv[0]*pn.b01 + Bu[2]*Bv[0]*pn.b10 + Bu[3]*Bv[0]*pn.b1  +
            Bu[0]*Bv[1]*pn.b03 + Bu[1]*Bv[1]*pn.b02 + Bu[2]*Bv[1]*pn.b13 + Bu[3]*Bv[1]*pn.b12 +
            Bu[0]*Bv[2]*pn.b30 + Bu[1]*Bv[2]*pn.b31 + Bu[2]*Bv[2]*pn.b20 + Bu[3]*Bv[2]*pn.b21 +
            Bu[0]*Bv[3]*pn.b3  + Bu[1]*Bv[3]*pn.b32 + Bu[2]*Bv[3]*pn.b23 + Bu[3]*Bv[3]*pn.b2;

	Bu = Bn(u);
	Bv = Bn(v);

	pnNorm=	Bu[0]*Bv[0]*pn.n0  + Bu[1]*Bv[0]*pn.n01		+ Bu[2]*Bv[0]*pn.n1 +
            Bu[0]*Bv[1]*pn.n30 + Bu[1]*Bv[1]*pn.n0123	+ Bu[2]*Bv[1]*pn.n12 +
            Bu[0]*Bv[2]*pn.n3  + Bu[1]*Bv[2]*pn.n23		+ Bu[2]*Bv[2]*pn.n2;
	
	*/
    //pnPos += params[0]*pnNorm; //debug

	  vec3 pnPos = interpolate3(pn.b03, pn.b00, pn.b30, pn.b33);
	  vec4 U4 = aa_pn_quads_basis (u);
	  vec4 V4 = aa_pn_quads_basis (v);
	  //*
	  mat4 Bx = mat4 (pn.b00.x, pn.b01.x, pn.b02.x, pn.b03.x, pn.b10.x, pn.b11.x, pn.b12.x, pn.b13.x, pn.b20.x, pn.b21.x, pn.b22.x, pn.b23.x, pn.b30.x, pn.b31.x, pn.b32.x, pn.b33.x);
	  mat4 By = mat4 (pn.b00.y, pn.b01.y, pn.b02.y, pn.b03.y, pn.b10.y, pn.b11.y, pn.b12.y, pn.b13.y, pn.b20.y, pn.b21.y, pn.b22.y, pn.b23.y, pn.b30.y, pn.b31.y, pn.b32.y, pn.b33.y);
	  mat4 Bz = mat4 (pn.b00.z, pn.b01.z, pn.b02.z, pn.b03.z, pn.b10.z, pn.b11.z, pn.b12.z, pn.b13.z, pn.b20.z, pn.b21.z, pn.b22.z, pn.b23.z, pn.b30.z, pn.b31.z, pn.b32.z, pn.b33.z);

	  mat4 M4 = mat4 (+1, -3, +3, -1, 0, +3, -6, +3, 0, 0, +3, -3, 0, 0, 0, +1);
	  mat4 M4t = transpose (M4);

	  

	  mat4 M4x = M4 * Bx * M4t;
	  mat4 M4y = M4 * By * M4t;
	  mat4 M4z = M4 * Bz * M4t;

	  pnPos.x = dot (U4, M4x * V4);
	  pnPos.y = dot (U4, M4y * V4);
	  pnPos.z = dot (U4, M4z * V4);
	  //*/

	  vec3 pnNorm;
	  mat3 Nx = mat3 (pn.n00.x, pn.n01.x, pn.n02.x, pn.n10.x, pn.n11.x, pn.n12.x, pn.n20.x, pn.n21.x, pn.n22.x);
	  mat3 Ny = mat3 (pn.n00.y, pn.n01.y, pn.n02.y, pn.n10.y, pn.n11.y, pn.n12.y, pn.n20.y, pn.n21.y, pn.n22.y);
	  mat3 Nz = mat3 (pn.n00.z, pn.n01.z, pn.n02.z, pn.n10.z, pn.n11.z, pn.n12.z, pn.n20.z, pn.n21.z, pn.n22.z);

	  mat3 M3 = mat3 (+1, -2, +1, 0, +2, -2, 0, 0, +1);
	  mat3 M3t = transpose (M3);

	  vec3 U3 = U4.xyz;
	  vec3 V3 = V4.xyz;

	  mat3 M3x = M3 * Nx * M3t;
	  mat3 M3y = M3 * Ny * M3t;
	  mat3 M3z = M3 * Nz * M3t;

	  pnNorm.x = dot (U3, M3x * V3);
	  pnNorm.y = dot (U3, M3y * V3);
	  pnNorm.z = dot (U3, M3z * V3);


	gpos = pnPos;
	gnormal = pnNorm;

	//pnPos += 0.0*disp(pnPos.xyz, vec3(10.0+time, 10.0, 20.0+time));

	gl_Position = vec4(pnPos, 1.0);

	frame_id_g = interpolate1(frame_id_te[0], frame_id_te[1], frame_id_te[3], frame_id_te[2]);
	
}

#version 430 core
layout (vertices = 4) out;

layout(location = 4) uniform float params[10];

struct pnQuad
{
	vec3 b0, b1, b2, b3;
	
	vec3 b01, b02, b03;
	vec3 b10, b12, b13;
	vec3 b20, b21, b23;
	vec3 b30, b31, b32;

	vec3 n0, n1, n2, n3;
	vec3 n01, n12, n23, n30;
	vec3 n0123;

	vec3 m0, m1, m2, m3;
};

in float frame_id_tc[];
out float frame_id_te[];

patch out pnQuad pn;
vec3 N[4];
vec3 P[4];

float wij(int i, int j)
{
	return dot(P[j] - P[i], N[i]);
}
 
float vij(int i, int j)
{
	vec3 Pj_minus_Pi = P[j]	- P[i];
	vec3 Ni_plus_Nj  = N[i] + N[j];
	return 2.0*dot(Pj_minus_Pi, Ni_plus_Nj)/dot(Pj_minus_Pi, Pj_minus_Pi);
}

vec3 bij(int i, int j)
{
	return (2.0*P[i] + P[j] - wij(i,j)*N[i])/3.0;
}

vec3 calcNormal(vec3 p0, vec3 p1, vec3 p2)
{
	return cross(p1-p0, p2-p0);
}

vec3 calcNormalX(int i, int j, int k)
{
	vec3 n = cross(gl_in[j].gl_Position.xyz-gl_in[i].gl_Position.xyz, gl_in[k].gl_Position.xyz-gl_in[i].gl_Position.xyz);
	return normalize(n);
	//return n;
}




void main()
{
	gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
	frame_id_te[gl_InvocationID] = frame_id_tc[gl_InvocationID];

	if(gl_InvocationID == 0)
	{
		//set tessellation levels
		const int utess = 8;
		const int vtess = 8;
		gl_TessLevelOuter[0] = utess;
		gl_TessLevelOuter[1] = vtess;
		gl_TessLevelOuter[2] = utess;
		gl_TessLevelOuter[3] = vtess;

		gl_TessLevelInner[0] = utess;
		gl_TessLevelInner[1] = vtess;
		
	
		P[0] = gl_in[2].gl_Position.xyz;
		P[1] = gl_in[3].gl_Position.xyz;
		P[2] = gl_in[5].gl_Position.xyz;
		P[3] = gl_in[4].gl_Position.xyz;

		/*
		N[0] = normalize(calcNormalX(2,0,3) + calcNormalX(2,3,4));
		N[1] = normalize(calcNormalX(3,2,1) + calcNormalX(3,5,2));
		N[2] = normalize(calcNormalX(5,4,3) + calcNormalX(5,7,4));
		N[3] = normalize(calcNormalX(4,2,5) + calcNormalX(4,5,6));
		*/

		pn.m0 = P[3]-P[0];
		pn.m1 = P[2]-P[1];
		pn.m2 = gl_in[7].gl_Position.xyz - gl_in[5].gl_Position.xyz;
		pn.m3 = gl_in[6].gl_Position.xyz - gl_in[4].gl_Position.xyz;
		
		N[0] = normalize(calcNormalX(2,0,3) + calcNormalX(2,3,4));
		N[1] = normalize(calcNormalX(3,2,1) + calcNormalX(3,5,2));
		N[2] = normalize(calcNormalX(5,4,3) + calcNormalX(5,7,4));
		N[3] = normalize(calcNormalX(4,2,5) + calcNormalX(4,5,6));


		pn.b0 = P[0]; pn.b1 = P[1]; pn.b2 = P[2]; pn.b3 = P[3];
		pn.n0 = N[0]; pn.n1 = N[1]; pn.n2 = N[2]; pn.n3 = N[3];

		pn.b01 = bij(0,1);
		pn.b03 = bij(0,3); 
		
		pn.b10 = bij(1,0); 
		pn.b12 = bij(1,2); 
		
		pn.b21 = bij(2,1); 
		pn.b23 = bij(2,3); 
		
		pn.b30 = bij(3,0); 
		pn.b32 = bij(3,2); 

		float sigma = 0.5;

		vec3 q = pn.b01 + pn.b03 + pn.b10 + pn.b12 + pn.b21 + pn.b23 + pn.b30 + pn.b32;
		vec3 e0 = (2.0*(pn.b01 + pn.b03 + q)-(pn.b21 + pn.b23))/18.0;
		vec3 e1 = (2.0*(pn.b12 + pn.b10 + q)-(pn.b32 + pn.b30))/18.0;
		vec3 e2 = (2.0*(pn.b23 + pn.b21 + q)-(pn.b03 + pn.b01))/18.0;
		vec3 e3 = (2.0*(pn.b30 + pn.b32 + q)-(pn.b10 + pn.b12))/18.0;

		vec3 v0 = (4.0*P[0] + 2.0*(P[3]+P[1]) + P[2])/9.0;
		vec3 v1 = (4.0*P[1] + 2.0*(P[0]+P[2]) + P[3])/9.0;
		vec3 v2 = (4.0*P[2] + 2.0*(P[1]+P[3]) + P[0])/9.0;
		vec3 v3 = (4.0*P[3] + 2.0*(P[2]+P[0]) + P[1])/9.0;

		

		/*
		pn.b02 = (1.0-sigma)*e0 + sigma*v0;
		pn.b13 = (1.0-sigma)*e1 + sigma*v1;
		pn.b20 = (1.0-sigma)*e2 + sigma*v2;
		pn.b31 = (1.0-sigma)*e3 + sigma*v3;
		*/

		pn.b02 = (1.0+sigma)*e0 - sigma*v0;
		pn.b13 = (1.0+sigma)*e1 - sigma*v1;
		pn.b20 = (1.0+sigma)*e2 - sigma*v2;
		pn.b31 = (1.0+sigma)*e3 - sigma*v3;


		pn.n01 = normalize(N[0] + N[1] - vij(0,1)*(P[1]-P[0]));
		pn.n12 = normalize(N[1] + N[2] - vij(1,2)*(P[2]-P[1]));
		pn.n23 = normalize(N[2] + N[3] - vij(2,3)*(P[3]-P[2]));
		pn.n30 = normalize(N[3] + N[0] - vij(3,0)*(P[0]-P[3]));
		pn.n0123 = (2.0*(pn.n01 + pn.n12 + pn.n23 + pn.n30) + pn.n0 + pn.n1 + pn.n2 + pn.n3)/12.0;
	}
}
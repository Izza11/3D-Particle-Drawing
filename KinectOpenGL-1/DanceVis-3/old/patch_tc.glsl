#version 430 core
layout (vertices = 4) out;

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

vec3 calcNormal(vec3 p0, vec3 p1, vec3 p2)
{
	return cross(p1-p0, p2-p0);
}

vec3 calcNormalX(int i, int j, int k)
{
	vec3 n = cross(gl_in[j].gl_Position.xyz-gl_in[i].gl_Position.xyz, gl_in[k].gl_Position.xyz-gl_in[i].gl_Position.xyz);
	return -normalize(n);
}

vec3 bij(int i, int j)
{
	//pn.b01 = (2.0*P[0] + P[1] - wij(0,1)*N[0])/3.0;
	return (2.0*P[i] + P[j] - wij(i,j)*N[i])/3.0;
}

vec3 aa_pn_quads_boundary (vec3 p0, vec3 p1, vec3 n0)
{
  return (2.0 * p0 + p1 - dot (p1 - p0, n0) * n0) / 3.0;
}

vec3 aa_pn_quads_normal (vec3 p0, vec3 p1, vec3 n0, vec3 n1)
{
  return normalize (n0 + n1 - (2.0 * dot (p1 - p0, n0 + n1) / dot (p1 - p0, p1 - p0)) * (p1 - p0));
}


void main()
{
	gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
	frame_id_te[gl_InvocationID] = frame_id_tc[gl_InvocationID];

	if(gl_InvocationID == 0)
	{
		//set tessellation levels
		const int utess = 14;
		const int vtess = 14;
		gl_TessLevelOuter[0] = utess;
		gl_TessLevelOuter[1] = vtess;
		gl_TessLevelOuter[2] = utess;
		gl_TessLevelOuter[3] = vtess;

		gl_TessLevelInner[0] = utess;
		gl_TessLevelInner[1] = vtess;
		
	
		P[1] = gl_in[2].gl_Position.xyz;
		P[0] = gl_in[3].gl_Position.xyz;
		P[2] = gl_in[5].gl_Position.xyz;
		P[3] = gl_in[4].gl_Position.xyz;
		
		N[1] = normalize(calcNormalX(2,0,3) + calcNormalX(2,3,4));
		N[0] = normalize(calcNormalX(3,2,1) + calcNormalX(3,5,2));
		N[2] = normalize(calcNormalX(5,4,3) + calcNormalX(5,7,4));
		N[3] = normalize(calcNormalX(4,2,5) + calcNormalX(4,5,6));

		/*
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

		pn.b02 = (1.0+sigma)*e0 - sigma*v0;
		pn.b13 = (1.0+sigma)*e1 - sigma*v1;
		pn.b20 = (1.0+sigma)*e2 - sigma*v2;
		pn.b31 = (1.0+sigma)*e3 - sigma*v3;
		*/

		float aa_pn_quads_weight = +0.5;
		  pn.b00 = P[0];
		  pn.b03 = P[1];
		  pn.b30 = P[2];
		  pn.b33 = P[3];


		    pn.n00 = N[0];
			pn.n02 = N[1];
			pn.n20 = N[2];
			pn.n22 = N[3];

		  pn.b01 = aa_pn_quads_boundary (pn.b00, pn.b03, pn.n00);
		  pn.b02 = aa_pn_quads_boundary (pn.b03, pn.b00, pn.n02);
		  pn.b10 = aa_pn_quads_boundary (pn.b00, pn.b30, pn.n00);
		  pn.b13 = aa_pn_quads_boundary (pn.b03, pn.b33, pn.n02);
		  pn.b20 = aa_pn_quads_boundary (pn.b30, pn.b00, pn.n20);
		  pn.b23 = aa_pn_quads_boundary (pn.b33, pn.b03, pn.n22);
		  pn.b31 = aa_pn_quads_boundary (pn.b30, pn.b33, pn.n20);
		  pn.b32 = aa_pn_quads_boundary (pn.b33, pn.b30, pn.n22);

		  vec3 q = pn.b01 + pn.b02 + pn.b10 + pn.b13 + pn.b20 + pn.b23 + pn.b31 + pn.b32;

  pn.b11 = mix (2.0 * (pn.b01 + pn.b10 + q) - (pn.b23 + pn.b32), 8.0 * pn.b00 + 4.0 * (pn.b03 + pn.b30) + 2.0 * pn.b33, aa_pn_quads_weight) / 18.0;
  pn.b12 = mix (2.0 * (pn.b02 + pn.b13 + q) - (pn.b20 + pn.b31), 8.0 * pn.b03 + 4.0 * (pn.b00 + pn.b33) + 2.0 * pn.b30, aa_pn_quads_weight) / 18.0;
  pn.b21 = mix (2.0 * (pn.b20 + pn.b31 + q) - (pn.b02 + pn.b13), 8.0 * pn.b30 + 4.0 * (pn.b00 + pn.b33) + 2.0 * pn.b03, aa_pn_quads_weight) / 18.0;
  pn.b22 = mix (2.0 * (pn.b23 + pn.b32 + q) - (pn.b01 + pn.b10), 8.0 * pn.b33 + 4.0 * (pn.b03 + pn.b30) + 2.0 * pn.b00, aa_pn_quads_weight) / 18.0;

		  //pn.b11 = (1.0+aa_pn_quads_weight)*(2.0 * (pn.b01 + pn.b10 + q) - (pn.b23 + pn.b32)) / 18.0 + aa_pn_quads_weight*(8.0 * pn.b00 + 4.0 * (pn.b03 + pn.b30) + 2.0 * pn.b33) / 18.0;
		  //pn.b12 = (1.0+aa_pn_quads_weight)*(2.0 * (pn.b02 + pn.b13 + q) - (pn.b20 + pn.b31)) / 18.0 + aa_pn_quads_weight*(8.0 * pn.b03 + 4.0 * (pn.b00 + pn.b33) + 2.0 * pn.b30) / 18.0;
		  //pn.b21 = (1.0+aa_pn_quads_weight)*(2.0 * (pn.b20 + pn.b31 + q) - (pn.b02 + pn.b13)) / 18.0 + aa_pn_quads_weight*(8.0 * pn.b30 + 4.0 * (pn.b00 + pn.b33) + 2.0 * pn.b03) / 18.0;
		  //pn.b22 = (1.0+aa_pn_quads_weight)*(2.0 * (pn.b23 + pn.b32 + q) - (pn.b01 + pn.b10)) / 18.0 + aa_pn_quads_weight*(8.0 * pn.b33 + 4.0 * (pn.b03 + pn.b30) + 2.0 * pn.b00) / 18.0;

  pn.n01 = aa_pn_quads_normal (pn.b00, pn.b03, pn.n00, pn.n02);
  pn.n10 = aa_pn_quads_normal (pn.b00, pn.b30, pn.n00, pn.n20);
  pn.n12 = aa_pn_quads_normal (pn.b03, pn.b33, pn.n02, pn.n22);
  pn.n21 = aa_pn_quads_normal (pn.b30, pn.b33, pn.n20, pn.n22);
  pn.n11 = (2.0 * (pn.n01 + pn.n10 + pn.n12 + pn.n21) + (pn.n00 + pn.n02 + pn.n20 + pn.n22)) / 12.0;

		
	}
}
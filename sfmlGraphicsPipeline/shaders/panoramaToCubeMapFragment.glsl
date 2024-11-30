#version 400
out vec4 outColor;

in vec2 tcoord;
uniform sampler2D panoramaSampler;
uniform int face;

#define PI 3.14159265359

float atan2(in float y, in float x)
{
    return x == 0.0 ? sign(y)*PI/2 : atan(y, x);
}

vec3 tcoord_to_direction(int face, vec2 tc)
{
    return normalize(
           float(face == 0) * vec3(  1.f, tc.y, tc.x  ) +
           float(face == 1) * vec3( -1.f, tc.y, -tc.x ) +
           float(face == 2) * vec3( tc.x,  1.f, tc.y  ) +  
           float(face == 3) * vec3( tc.x, -1.f, -tc.y ) +  
           float(face == 4) * vec3( tc.x, tc.y, -1.f  ) +  
           float(face == 5) * vec3(-tc.x, tc.y, 1.f   )
    );
}

vec2 direction_to_spherical(vec3 dir)
{
	return vec2(
		atan2(dir.z, dir.x),
		acos(dir.y));
}

vec2 spherical_to_uv(vec2 phi_theta)
{
    return vec2(
        0.5 + 0.5 * phi_theta.x / PI,
        1 - phi_theta.y / PI);
}

void main()
{   
    vec3 dir = tcoord_to_direction(face, tcoord);
    vec2 phi_theta = direction_to_spherical(dir);
    vec2 uv = spherical_to_uv(phi_theta);

    outColor = texture(panoramaSampler, uv);
}
#version 400
out vec4 outColor;

in vec2 tcoord;
uniform samplerCube cubeMapSampler;

#define PI 3.14159265359

void main()
{   
    vec2 t = 1 - tcoord;
    float phi = t.x * 2 * PI + 1 * PI;
    phi = mod(phi, 2 * PI);

    float theta = t.y * PI;
    float sp = sin(phi);
    float st = sin(theta);
    float cp = cos(phi);
    float ct = cos(theta);
    vec3 dir = vec3(st*cp, ct, st*sp);

    vec4 sampled = texture(cubeMapSampler, dir);
    sampled.a = 1;
    outColor = sampled;
}
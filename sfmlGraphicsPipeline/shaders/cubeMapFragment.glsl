#version 400
out vec4 outColor;

in vec3 tcoords;

uniform samplerCube cubeMapSampler;

void main()
{   
    //outColor = vec4(0.5 + 0.5 * tcoords, 1);
    vec4 sampled = texture(cubeMapSampler, tcoords);
    sampled.a = 1;
    outColor = sampled;
}
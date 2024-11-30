#version 400
uniform mat4 projMat, viewMat, modelMat;

in vec3 vPosition;
in vec2 vTexCoord;
out vec2 surfel_texCoord;

uniform float time;

void main()
{
    // head is at (0,0,1), tail at (0,0,-1)
    float tail_weight = -0.5 * vPosition.z + 0.5;
    // Maybe the tail should swing more than the head : let's add more weight on the tail
    float delta_weight = (0.1 + 0.4 * pow(tail_weight,3));
    vec3 delta = vec3(sin(4*time + 2*vPosition.z), 0, 0);
    vec3 position = vPosition + delta_weight * delta;

    gl_Position = projMat*viewMat*modelMat*vec4(position, 1.0f);
    surfel_texCoord = vTexCoord;
}

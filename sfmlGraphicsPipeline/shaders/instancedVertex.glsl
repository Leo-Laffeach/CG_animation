#version 400

uniform mat4 projMat, viewMat, modelMat;
uniform mat3 NIT = mat3(1);

in vec3 vPosition;
in vec3 vNormal;
in vec4 vColor;
in vec4 instanceData;

out vec4 surfel_color;
out vec3 surfel_normal;

void main()
{
    float radius = instanceData.w;
    vec3 translation = instanceData.xyz;
    vec3 surfel_position = vPosition * radius + translation;
    surfel_normal = normalize( NIT * vNormal);
    surfel_color = vColor;
    gl_Position = projMat*viewMat*modelMat*vec4(surfel_position, 1.0f);
}

#version 400

uniform mat4 projMat, viewMat, modelMat;
uniform mat3 NIT;


in vec3 vPosition;

//Design choice : Color are always vec4
in vec4 vColor;
out vec4 color;
in vec3 vNormal;
out vec3 normal;
in vec2 vTexCoord;
out vec2 tcoord;

void main()
{
    gl_Position = projMat*viewMat*modelMat*vec4(vPosition, 1.0f);
    color = vColor;
    normal = NIT * vNormal;
    tcoord = vTexCoord;
}

#version 400

in vec4 surfel_color;
in vec3 surfel_normal;

out vec4 fragmentColor;

void main()
{
    fragmentColor = surfel_color;
}

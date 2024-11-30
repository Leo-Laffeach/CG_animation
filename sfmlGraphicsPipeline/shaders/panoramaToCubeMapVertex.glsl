#version 400

in vec2 vTexCoord;

out vec2 tcoord;

void main()
{
    tcoord = vTexCoord;
    gl_Position = vec4(vTexCoord, -1.0, 1.0);
}
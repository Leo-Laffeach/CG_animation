#version 400

//Design choice : Color are always vec4
in vec4 color;
out vec4 outColor;
in vec3 normal;
in vec2 tcoord;

uniform float time;

void main()
{
    //outColor = vec4(tcoord, 0,1);
    float f = 0.5 + 0.5 * sin(time);
    outColor = 0.5 * vec4(normal,1) + 0.5;
    //outColor = color;
}

#version 400

in vec3 vPosition;
in vec3 vNormal;

out vec3 tcoords;

uniform mat4 projMat, viewMat;

void main()
{
    tcoords = vNormal;
    mat4 viewMatTranslationFree = viewMat;
    viewMatTranslationFree[3] = vec4(0,0,0,1); // disable translations
    vec4 pos = projMat * viewMatTranslationFree * vec4(vPosition, 1.0);
    
    // Trick the depth buffer to see that this vertex
    // has the greatest depth value possible.
    // see https://learnopengl.com/Advanced-OpenGL/Cubemaps
    // for more information.
    gl_Position = pos.xyww; 
}
#version 400

uniform mat4 projMat, viewMat, modelMat;

in vec3 vPosition;
in vec4 vColor;
in vec3 vNormal;
in vec2 vTexCoord;

out vec2 surfel_tcoord;
out vec4 fragmentColor;

out vec3 normal;
out vec3 cameraPosition;
out vec3 surfacePosition;

void main()
{
    gl_Position = projMat*viewMat*modelMat*vec4(vPosition, 1.0f);
    fragmentColor = vColor;
    surfel_tcoord = vTexCoord;

    normal = normalize(transpose(inverse(mat3(modelMat))) * vNormal);
    surfacePosition = vec3(modelMat*vec4(vPosition,1.0f));
    cameraPosition = - vec3( viewMat[3] ) * mat3( viewMat );
}

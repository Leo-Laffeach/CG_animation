#version 400



in vec3 surfacePosition;
in vec4 fragmentColor;
in vec3 normal;
in vec3 cameraPosition;

in vec2 surfel_tcoord;

uniform sampler2D texSampler1;
uniform sampler2D texSampler2;

uniform float time;

out vec4 outColor;

void main()
{
    //Surface to camera vector
    vec3 viewDir = normalize( cameraPosition - surfacePosition );

    // Try other blending coefficent computation (e.g. with time)
    float blendingCoeff = 0.5;

    vec4 tex1_sample = texture2D(texSampler1, surfel_tcoord);
    vec4 tex2_sample = texture2D(texSampler2, surfel_tcoord);

    outColor = clamp(tex1_sample * tex2_sample,0,1);
}

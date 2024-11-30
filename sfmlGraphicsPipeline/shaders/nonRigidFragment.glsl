#version 400
uniform sampler2D texSampler;

in vec2 surfel_texCoord;
out vec4 outColor;

void main()
{
    vec4 textureColor = texture(texSampler, surfel_texCoord);
    outColor = textureColor;
}

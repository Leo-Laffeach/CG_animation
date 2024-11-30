#version 400
out vec4 outColor;

in vec2 tcoord;
uniform sampler2D texSampler;
uniform int ksize = 150;
uniform int csize = 1024;
uniform vec2 direction = vec2(0,1);

#define PI 3.14159265359

void main()
{   
    vec2 t = 0.5 + 0.5 * tcoord;

    // 2 pass gaussian blur borrowed from http://callumhay.blogspot.com/search/label/glsl
    // Incremental Gaussian Coefficent Calculation (See GPU Gems 3 pp. 877 - 889)
    float sigma = (ksize - 1) / 6.f;
    vec3 incrementalGaussian;
    incrementalGaussian.x = 1.0f / (sqrt(2.0f * PI) * sigma);
    incrementalGaussian.y = exp(-0.5f / (sigma * sigma));
    incrementalGaussian.z = incrementalGaussian.y * incrementalGaussian.y;

    vec4 avgValue = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    float coefficientSum = 0.0f;

    // Take the central sample first...
    avgValue += texture2D(texSampler, t.xy) * incrementalGaussian.x;
    coefficientSum += incrementalGaussian.x;
    incrementalGaussian.xy *= incrementalGaussian.yz;

    // Go through the remaining 8 vertical samples (4 on each side of the center)
    float blurSize = 1.f / csize;
    
    for (int i = 1; i <= ksize/2; i++) { 
        avgValue += texture2D(texSampler, t.xy - i * blurSize * direction) * incrementalGaussian.x;         
        avgValue += texture2D(texSampler, t.xy + i * blurSize * direction) * incrementalGaussian.x;         
        coefficientSum += 2 * incrementalGaussian.x;
        incrementalGaussian.xy *= incrementalGaussian.yz;
    }

    outColor = avgValue / coefficientSum;
}
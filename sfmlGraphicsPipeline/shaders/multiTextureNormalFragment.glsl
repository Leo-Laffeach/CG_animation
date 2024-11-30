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

vec4 angleDot(float dotprod, vec3 axis)
{
    float s = sqrt(0.5 - 0.5 * dotprod);
    float w = sqrt(0.5 + 0.5 * dotprod);
    return vec4(axis * s, w);
}

vec4 two_dirs_to_quat(vec3 v1, vec3 v2){
    vec3 axis = cross(v1, v2);
    float dotprod = dot(v1, v2);
    return angleDot(dotprod, axis);
}

vec4 qmul(vec4 q1, vec4 q2) {
	return vec4(
		q2.xyz * q1.w + q1.xyz * q2.w + cross(q1.xyz, q2.xyz),
		q1.w * q2.w - dot(q1.xyz, q2.xyz)
	);
}

vec3 rotate_vector(vec4 q, vec3 v) {
    vec4 qconj = q * vec4(-1, -1, -1, 1);
	return qmul(q, qmul(vec4(v, 0), qconj)).xyz;
}

void main(){
    // Normal deviation weight
    float normal_weight = 5 ;
    
    // get the texel colors
    vec4 tex_sample = texture2D(texSampler1, surfel_tcoord);
    vec3 normal_sample = texture2D(texSampler2, surfel_tcoord).rgb;

    // normalize the normal (careful, the xy components should be positive and negative)
    normal_sample = normal_sample - vec3(0.5,0.5, 0);
    normal_sample.xy = normal_weight * normal_sample.xy;
    normal_sample = normalize(normal_sample);

    // Make a quaternion to remap this modified normal to our normal direction.
    // The blue color corresponds to untouched normal so we want the rotation that
    // make the z axis align to our surfel normal
    vec4 q = two_dirs_to_quat(vec3(0,0,1), normal);
    vec3 tilted_normal = rotate_vector(q, normal_sample);

    // Let's harcode some diffuse shading with two directional lights.
    // We use the tilted normal instead of the surfel normal for the diffuse factor computation
    vec3 light1_dir = normalize(vec3(sin(time), 0.5 * cos(0.5*time), cos(time)));
    vec3 light2_dir = normalize(vec3(-1, sin(0.75*time), -1));
    vec4 light1_color = vec4(0.7,0.7,0.9,1); // blue-ish
    vec4 light2_color = vec4(0.5,0.4,0.3,1); // red-ish

    float diffuse1_factor = max(0,dot(-light1_dir, tilted_normal));
    float diffuse2_factor = max(0,dot(-light2_dir, tilted_normal));

    vec4 illumn = light1_color * diffuse1_factor + light2_color * diffuse2_factor;

    // result = easy hack to add illusion of depth to a polygon
    outColor = illumn * tex_sample;
}

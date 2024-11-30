#version 400

//Structure definition for Material, DirectionalLight, PointLight and SpotLight
//Parameters are exactly the same as the corresponding C++ classes
//Refer to the C++ documentation for more information


// Surfel: a SURFace ELement. All coordinates are in world space
in vec3 surfel_position;
in vec4 surfel_color;
in vec3 surfel_normal;

// Camera position in world space
in vec3 cameraPosition;

// Resulting color of the fragment shader
out vec4 outColor;

void main()
{
    vec3 ambient = vec3(0.1);
    vec3 diffuse = vec3(1);

    vec3 surfel_to_camera = normalize( cameraPosition - surfel_position );
    vec3 surfel_to_light = normalize(vec3(1));
    float diffuse_factor = max(dot(surfel_to_light, surfel_normal), 0);

    float first_diffuse = 1 * float(diffuse_factor > 0.8);    
    float second_diffuse = 0.8 * float(diffuse_factor <= 0.8) * float(diffuse_factor > 0.4);    
    float third_diffuse = 0.3 * float(diffuse_factor <= 0.4) * float(diffuse_factor > 0.0);   

    vec3 cartoon_color = diffuse * (first_diffuse + second_diffuse + third_diffuse);

    float cell_factor =  max(dot(surfel_to_camera, surfel_normal), 0);
    cell_factor = float(cell_factor>0.35);

    cartoon_color = cartoon_color * cell_factor;

    outColor = vec4(cartoon_color,1.0);
}

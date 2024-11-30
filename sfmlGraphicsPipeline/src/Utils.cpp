#include "./../include/Utils.hpp"
#include "./../include/log.hpp"

#include <chrono>
#include <random>
#include <glm/gtx/color_space.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace std;

// A random engine initialized on the current time.
// This way, the random numbers generated won't be the same at each execution.
static default_random_engine generator(chrono::system_clock::now().time_since_epoch().count() );

float random(float a, float b)
{
    uniform_real_distribution<> distribution(a,b);
    return distribution(generator);
}

glm::vec4 randomColor()
{
    uniform_real_distribution<> distribution(0,360);
    float hue = distribution(generator), saturation = 1.0, black = 1.0;
    glm::vec3 hsvColor( hue, saturation, black);
    glm::vec3 rgbColor = glm::rgbColor(hsvColor);
    return glm::vec4(rgbColor[0], rgbColor[1], rgbColor[2], 1.0);
}

glm::vec4 getColor(float factor, float low, float high)
{
    float hue = 0.0, saturation = 1.0, black = 1.0;
    if(factor<low) hue = 240.0;
    else if(factor>high) hue = 0.0;
    else hue = 240 + (factor-low)/(high-low)*(0.0-240.0);
    glm::vec3 hsvColor( hue, saturation, black);
    glm::vec3 rgbColor = glm::rgbColor(hsvColor);
    return glm::vec4(rgbColor[0], rgbColor[1], rgbColor[2], 1.0);
}

void getUnitPlane(vector<glm::vec3>& positions, vector<glm::vec3>& normals, vector<glm::vec2> &tcoords)
{
    positions.resize(6, glm::vec3(0.0,0.0,0.0));
    tcoords.resize(6, glm::vec2(0.0,0.0));
    normals.resize(6, glm::vec3(0.0,0.0,1.0));

    //First triangle
    positions[0] =glm::vec3(-0.5,-0.5,0.0);
    positions[1] =glm::vec3(0.5,-0.5,0.0);
    positions[2] =glm::vec3(0.5,0.5,0.0);

    //TODO
    tcoords[0] =glm::vec2(0.0,0.0);
    tcoords[1] =glm::vec2(1.0,0.0);
    tcoords[2] =glm::vec2(1.0,1.0);

    //Second triangle
    positions[3] =glm::vec3(-0.5,-0.5,0.0);
    positions[4] =glm::vec3(0.5,0.5,0.0);
    positions[5] =glm::vec3(-0.5,0.5,0.0);

    tcoords[3] =glm::vec2(0.0,0.0);
    tcoords[4] =glm::vec2(1.0,1.0);
    tcoords[5] =glm::vec2(0.0,1.0);
}

void getIndexedUnitPlane(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> &tcoords, std::vector<glm::uvec3>& indices)
{
    positions.resize(4);
    tcoords.resize(4);
    normals.resize(4, glm::vec3(0,0,1));
    indices.resize(2);
    
    for (int i=0;i<2;++i){
        for(int j=0;j<2;++j){
            size_t index = 2 * i + j;
            positions[index] = glm::vec3(-0.5 + i,-0.5 + j, 0.0);
            tcoords[index] = glm::vec2(i, j);
        }
    }
    indices[0] = glm::uvec3(0,1,2);
    indices[1] = glm::uvec3(1,2,3);
}


void getUnitIndexedCube(vector<glm::vec3>& positions, vector<glm::vec3>& normals, vector<glm::uvec3>& indices)
{
    positions.resize(0);
    normals.resize(0);
    indices.resize(0);

    positions.push_back(glm::vec3(-0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(-0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(-0.5, -0.5, 0.5));
    positions.push_back(glm::vec3(-0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(0.5, -0.5, 0.5));

    for(const glm::vec3& x : positions)
    {
        normals.push_back( glm::normalize(x) );
    }

    indices.push_back(glm::uvec3(0, 2, 1));
    indices.push_back(glm::uvec3(0, 3, 2));
    indices.push_back(glm::uvec3(1, 6, 7));
    indices.push_back(glm::uvec3(1, 2, 6));
    indices.push_back(glm::uvec3(5, 4, 7));
    indices.push_back(glm::uvec3(5, 7, 6));
    indices.push_back(glm::uvec3(4, 3, 0));
    indices.push_back(glm::uvec3(4, 5, 3));
    indices.push_back(glm::uvec3(3, 6, 2));
    indices.push_back(glm::uvec3(3, 5, 6));
    indices.push_back(glm::uvec3(4, 0, 1));
    indices.push_back(glm::uvec3(4, 1, 7));
}


void getUnitCube(vector<glm::vec3>& positions, vector<glm::vec3>& normals, vector<glm::vec2>& tcoords)
{
    // Make sure vectors are empty
    positions.clear();
    normals.clear();
    tcoords.clear();

    //Face 1
    positions.push_back(glm::vec3(-0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(0.5, -0.5, -0.5));

    normals.push_back(glm::vec3(0.0,0.0,-1.0));
    normals.push_back(glm::vec3(0.0,0.0,-1.0));
    normals.push_back(glm::vec3(0.0,0.0,-1.0));

    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(1.0,0.0));
    tcoords.push_back(glm::vec2(1.0,1.0));

    //Face 2
    positions.push_back(glm::vec3(-0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(-0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(0.5, 0.5, -0.5));

    normals.push_back(glm::vec3(0.0,0.0,-1.0));
    normals.push_back(glm::vec3(0.0,0.0,-1.0));
    normals.push_back(glm::vec3(0.0,0.0,-1.0));

    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(0.0,0.0));
    tcoords.push_back(glm::vec2(1.0,0.0));

    //Face 3
    positions.push_back(glm::vec3(0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(0.5, -0.5, 0.5));

    normals.push_back(glm::vec3(1.0,0.0,0.0));
    normals.push_back(glm::vec3(1.0,0.0,0.0));
    normals.push_back(glm::vec3(1.0,0.0,0.0));

    tcoords.push_back(glm::vec2(0.0,0.0));
    tcoords.push_back(glm::vec2(1.0,1.0));
    tcoords.push_back(glm::vec2(0.0,1.0));

    //Face 4
    positions.push_back(glm::vec3(0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(0.5, 0.5, 0.5));

    normals.push_back(glm::vec3(1.0,0.0,0.0));
    normals.push_back(glm::vec3(1.0,0.0,0.0));
    normals.push_back(glm::vec3(1.0,0.0,0.0));

    tcoords.push_back(glm::vec2(0.0,0.0));
    tcoords.push_back(glm::vec2(1.0,0.0));
    tcoords.push_back(glm::vec2(1.0,1.0));

    //Face 5
    positions.push_back(glm::vec3(-0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(-0.5, -0.5, 0.5));
    positions.push_back(glm::vec3(0.5, -0.5, 0.5));

    normals.push_back(glm::vec3(0.0,0.0,1.0));
    normals.push_back(glm::vec3(0.0,0.0,1.0));
    normals.push_back(glm::vec3(0.0,0.0,1.0));

    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(0.0,0.0));
    tcoords.push_back(glm::vec2(1.0,0.0));

    //Face 6
    positions.push_back(glm::vec3(-0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(0.5, -0.5, 0.5));
    positions.push_back(glm::vec3(0.5, 0.5, 0.5));

    normals.push_back(glm::vec3(0.0,0.0,1.0));
    normals.push_back(glm::vec3(0.0,0.0,1.0));
    normals.push_back(glm::vec3(0.0,0.0,1.0));

    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(1.0,0.0));
    tcoords.push_back(glm::vec2(1.0,1.0));

    //Face 7
    positions.push_back(glm::vec3(-0.5, -0.5, 0.5));
    positions.push_back(glm::vec3(-0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(-0.5, -0.5, -0.5));

    normals.push_back(glm::vec3(-1.0,0.0,0.0));
    normals.push_back(glm::vec3(-1.0,0.0,0.0));
    normals.push_back(glm::vec3(-1.0,0.0,0.0));

    tcoords.push_back(glm::vec2(1.0,1.0));
    tcoords.push_back(glm::vec2(0.0,0.0));
    tcoords.push_back(glm::vec2(1.0,0.0));

    //Face 8
    positions.push_back(glm::vec3(-0.5, -0.5, 0.5));
    positions.push_back(glm::vec3(-0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(-0.5, 0.5, -0.5));

    normals.push_back(glm::vec3(-1.0,0.0,0.0));
    normals.push_back(glm::vec3(-1.0,0.0,0.0));
    normals.push_back(glm::vec3(-1.0,0.0,0.0));

    tcoords.push_back(glm::vec2(1.0,1.0));
    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(0.0,0.0));

    //Face 9
    positions.push_back(glm::vec3(-0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(0.5, 0.5, -0.5));

    normals.push_back(glm::vec3(0.0,1.0,0.0));
    normals.push_back(glm::vec3(0.0,1.0,0.0));
    normals.push_back(glm::vec3(0.0,1.0,0.0));

    tcoords.push_back(glm::vec2(1.0,0.0));
    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(0.0,0.0));

    //Face 10
    positions.push_back(glm::vec3(-0.5, 0.5, -0.5));
    positions.push_back(glm::vec3(-0.5, 0.5, 0.5));
    positions.push_back(glm::vec3(0.5, 0.5, 0.5));

    normals.push_back(glm::vec3(0.0,1.0,0.0));
    normals.push_back(glm::vec3(0.0,1.0,0.0));
    normals.push_back(glm::vec3(0.0,1.0,0.0));

    tcoords.push_back(glm::vec2(1.0,0.0));
    tcoords.push_back(glm::vec2(1.0,1.0));
    tcoords.push_back(glm::vec2(0.0,1.0));

    //Face 11
    positions.push_back(glm::vec3(-0.5, -0.5, 0.5));
    positions.push_back(glm::vec3(-0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(0.5, -0.5, -0.5));

    normals.push_back(glm::vec3(0.0,-1.0,0.0));
    normals.push_back(glm::vec3(0.0,-1.0,0.0));
    normals.push_back(glm::vec3(0.0,-1.0,0.0));

    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(0.0,0.0));
    tcoords.push_back(glm::vec2(1.0,0.0));

    //Face 12
    positions.push_back(glm::vec3(-0.5, -0.5, 0.5));
    positions.push_back(glm::vec3(0.5, -0.5, -0.5));
    positions.push_back(glm::vec3(0.5, -0.5, 0.5));

    normals.push_back(glm::vec3(0.0,-1.0,0.0));
    normals.push_back(glm::vec3(0.0,-1.0,0.0));
    normals.push_back(glm::vec3(0.0,-1.0,0.0));

    tcoords.push_back(glm::vec2(0.0,1.0));
    tcoords.push_back(glm::vec2(1.0,0.0));
    tcoords.push_back(glm::vec2(1.0,1.0));
}

void getUnitCylinder(vector<glm::vec3> &positions, vector<glm::vec3> &normals, std::vector<glm::vec2> & tcoords, unsigned int slices, bool vertex_normals)
{
    size_t number_of_triangles = slices * 4; // four triangles per slice
    size_t number_of_vertices = number_of_triangles * 3; // three vertices per triangle (unindexed version)
    float angle_step = 2.0 * M_PI / double( slices );

    positions.resize(number_of_vertices, glm::vec3(0.0,0.0,0.0));
    normals.resize(number_of_vertices, glm::vec3(0.0,0.0,0.0));
    tcoords.resize(number_of_vertices, glm::vec2(0));

    float previous_angle = (slices - 1) * angle_step;
    float angle = 0.0;
    float previous_sin = std::sin(previous_angle);
    float previous_cos = std::cos(previous_angle);
    float sin = 0.0; // sin (0) = 0
    float cos = 1.0; // cos (0) = 1 

    float half_angle;
    float half_cos;
    float half_sin;

    for( size_t i = 0; i < slices; ++ i)
    {
        size_t voffset = 12 * i; // 4 x 3 = 12 vertices per slice

        // Positions

        // top triangle
        positions[ voffset +  0 ] = glm::vec3(0.0, 0.0, 1.0);
        positions[ voffset +  1 ] = glm::vec3(cos, sin, 1.0);
        positions[ voffset +  2 ] = glm::vec3(previous_cos, previous_sin, 1.0);

        // side triangles
        positions[ voffset +  3 ] = glm::vec3(previous_cos, previous_sin, 1.0);
        positions[ voffset +  4 ] = glm::vec3(cos, sin, 1.0);
        positions[ voffset +  5 ] = glm::vec3(previous_cos, previous_sin, 0.0);
        positions[ voffset +  6 ] = glm::vec3(cos, sin, 0.0);
        positions[ voffset +  7 ] = glm::vec3(previous_cos, previous_sin, 0.0);
        positions[ voffset +  8 ] = glm::vec3(cos, sin, 1.0);

        // bottom triangle
        positions[ voffset +  9  ] = glm::vec3(0.0, 0.0, 0.0);
        positions[ voffset +  10 ] = glm::vec3(previous_cos, previous_sin, 0.0);
        positions[ voffset +  11 ] = glm::vec3(cos, sin, 0.0);

        // Normals

        // top triangle
        normals[ voffset +  0 ] = glm::vec3(0.0, 0.0, 1.0);
        normals[ voffset +  1 ] = glm::vec3(0.0, 0.0, 1.0);
        normals[ voffset +  2 ] = glm::vec3(0.0, 0.0, 1.0);
        
        // side triangles
        if (vertex_normals){
            // Per vertex normals
            normals[ voffset +  3 ] = glm::vec3(previous_cos, previous_sin, 0.0);
            normals[ voffset +  4 ] = glm::vec3(cos, sin, 0.0);
            normals[ voffset +  5 ] = glm::vec3(previous_cos, previous_sin, 0.0);
            normals[ voffset +  6 ] = glm::vec3(cos, sin, 0.0);
            normals[ voffset +  7 ] = glm::vec3(previous_cos, previous_sin, 0.0);
            normals[ voffset +  8 ] = glm::vec3(cos, sin, 0.0);
        }else{
            // Per triangle normals
            half_angle = previous_angle + 0.5*angle_step;
            half_cos = std::cos(half_angle);
            half_sin = std::sin(half_angle);
            for (int j = 3; j < 9; j++){ 
                normals[ voffset +  j ] = glm::vec3(half_cos, half_sin, 0.0); // cos*cos + sin*sin = 1
            }
        }

        normals[ voffset +  9 ] = glm::vec3(0.0, 0.0, -1.0);
        normals[ voffset + 10 ] = glm::vec3(0.0, 0.0, -1.0);
        normals[ voffset + 11 ] = glm::vec3(0.0, 0.0, -1.0);

        // Actualise sin and cos
        previous_angle = angle;
        angle = i == slices - 1 ? 0 : angle + angle_step;
        previous_sin = sin; 
        previous_cos = cos; 
        sin = std::sin(angle);
        cos = std::cos(angle);

        // Texture coordinates (don't modify, might be used later)
        
        float current_u = i /(float)slices; 
        float next_u = (i+1) /(float)slices; 

        // top triangle
        tcoords[voffset + 0] = glm::vec2(current_u, 0);
        tcoords[voffset + 1] = glm::vec2(current_u, 0.33);
        tcoords[voffset + 2] = glm::vec2(next_u,0.33);

        // side triangles
        tcoords[voffset + 3] = glm::vec2(next_u, 0.33);
        tcoords[voffset + 4] = glm::vec2(current_u, 0.33);
        tcoords[voffset + 5] = glm::vec2(current_u,0.66);
        tcoords[voffset + 6] = glm::vec2(next_u, 0.33);
        tcoords[voffset + 7] = glm::vec2(next_u, 0.66);
        tcoords[voffset + 8] = glm::vec2(current_u,0.66);

        // bottom triangle
        tcoords[voffset + 9] = glm::vec2(current_u, 1);
        tcoords[voffset + 10] = glm::vec2(next_u, 0.66);
        tcoords[voffset + 11] = glm::vec2(current_u,0.66);
    }
}


void getUnitIndexedCylinder(vector<glm::vec3> & positions, vector<glm::vec3> & normals, vector<glm::uvec3> & indices, unsigned int slices)
{
    size_t number_of_vertices = slices * 2 + 2;
    size_t number_of_triangles = slices * 4;
    float angle_step = 2.0 * M_PI / (float)slices;

    positions.resize(number_of_vertices, glm::vec3(0, 0, 0));
    normals.resize(number_of_vertices, glm::vec3(0, 0, 0));
    indices.resize(number_of_triangles, glm::vec3{0, 0, 0});

    float angle = 0;
    float sin = 0;
    float cos = 1;

    positions[0] = glm::vec3(0, 0, 1);
    positions[1] = glm::vec3(0, 0, -1);
    normals[0] = glm::vec3(0, 0, 1);
    normals[1] = glm::vec3(0, 0, -1);

    for( size_t i = 0; i < slices;
            ++i,
            angle += angle_step,
            cos = std::cos(angle),
            sin = std::sin(angle) )
    {
        size_t voffset = 2 * i + 2;
        size_t toffset = 4 * i;
        positions[ voffset +  0 ] = glm::vec3{cos, sin, 1};
        positions[ voffset +  1 ] = glm::vec3{cos, sin, -1};

        normals[ voffset +  0 ] = glm::vec3{cos, sin, 0};
        normals[ voffset +  1 ] = glm::vec3{cos, sin, 0};

        size_t i_next = i == slices - 1 ? 0 : i + 1;
        size_t voffset_next = 2 * i_next + 2;

        indices[toffset + 0] = glm::uvec3{voffset + 0, voffset_next + 0 , 0};
        indices[toffset + 1] = glm::uvec3{voffset + 0, voffset_next + 0 , voffset + 1};
        indices[toffset + 2] = glm::uvec3{voffset_next + 1, voffset_next + 0 , voffset + 1};
        indices[toffset + 3] = glm::uvec3{voffset_next + 1, 1 , voffset + 1};
    }
}

void getUnitSphere(vector<glm::vec3>& positions, vector<glm::vec3>& normals, vector<glm::vec2> & tcoords, unsigned int strips, unsigned int slices, bool vertex_normals)
{
    float radius=1.0;
    glm::vec3 center(0.0,0.0,0.0);

    unsigned int vertexNumber = slices*strips*6;
    positions.resize(vertexNumber);
    normals.resize(vertexNumber);
    tcoords.resize(vertexNumber);

    for(int i=0; i<slices; ++i)
    {
        float u = i/(float)slices;
        float next_u = (i+1)/(float)slices;
        for(int j=0; j<strips; ++j)
        {
            float v = j/(float)strips;
            float next_v = (j+1)/(float)strips;

            float curr_theta = i*(2.0*M_PI/(float)slices);
            float curr_phi = j*(M_PI/(float)strips);

            float next_theta = (i+1)*(2.0*M_PI/(float)slices);
            float next_phi = (j+1)*(M_PI/(float)strips);

            float cos_theta = cos(curr_theta);
            float sin_theta = sin(curr_theta);
            float cos_phi = cos(curr_phi);
            float sin_phi = sin(curr_phi);
            float cos_next_theta = cos(next_theta);
            float sin_next_theta = sin(next_theta);
            float cos_next_phi = cos(next_phi);
            float sin_next_phi = sin(next_phi);

            glm::vec3 current_normal = glm::vec3(cos_theta*sin_phi, sin_theta*sin_phi, cos_phi);
            glm::vec3 next_theta_normal = glm::vec3(cos_next_theta*sin_phi, sin_next_theta*sin_phi, cos_phi);
            glm::vec3 next_phi_normal = glm::vec3(cos_theta*sin_next_phi, sin_theta*sin_next_phi, cos_next_phi);
            glm::vec3 next_both_normal = glm::vec3(cos_next_theta*sin_next_phi, sin_next_theta*sin_next_phi, cos_next_phi);
            
            size_t step =  i * strips + j;

            positions[6 * step + 0] = center + radius * current_normal;
            positions[6 * step + 1] = center + radius * next_theta_normal;
            positions[6 * step + 2] = center + radius * next_both_normal;

            positions[6 * step + 3] = center + radius * current_normal;
            positions[6 * step + 4] = center + radius * next_both_normal;
            positions[6 * step + 5] = center + radius * next_phi_normal;

            tcoords[6 * step + 0] = glm::vec2(u, v);
            tcoords[6 * step + 1] = glm::vec2(next_u, v);
            tcoords[6 * step + 2] = glm::vec2(next_u, next_v);

            tcoords[6 * step + 3] = glm::vec2(u, v);
            tcoords[6 * step + 4] = glm::vec2(next_u, next_v);
            tcoords[6 * step + 5] = glm::vec2(u, next_v);

            if (!vertex_normals){
                glm::vec3 t1_edge1 = glm::normalize(next_theta_normal - current_normal);
                glm::vec3 t1_edge2 = glm::normalize(next_both_normal - current_normal);
                glm::vec3 t2_edge1 = glm::normalize(next_both_normal - current_normal);
                glm::vec3 t2_edge2 = glm::normalize(next_phi_normal - current_normal);
                glm::vec3 t1_normal = glm::normalize(glm::cross(t1_edge1, t1_edge2));
                glm::vec3 t2_normal = glm::normalize(glm::cross(t2_edge1, t2_edge2));
                glm::vec3 t1_mean_normal = glm::normalize(current_normal + next_theta_normal + next_both_normal);
                glm::vec3 t2_mean_normal = glm::normalize(current_normal + next_both_normal + next_phi_normal);
                if(glm::dot(t1_normal, t1_mean_normal) < 0)
                    t1_normal *= -1;
                if(glm::dot(t2_normal, t2_mean_normal) < 0)
                    t2_normal *= -1;   
                for (size_t k=0; k<=5; ++k)
                    normals[6 * step + k] = k <= 2 ? t1_normal : t2_normal;
            }else{
                normals[6 * step + 0] = current_normal;
                normals[6 * step + 1] = next_theta_normal;
                normals[6 * step + 2] = next_both_normal;

                normals[6 * step + 3] = current_normal;
                normals[6 * step + 4] = next_both_normal;
                normals[6 * step + 5] = next_phi_normal;
            }
        }
    }
}

void getUnitIndexedSphere(vector<glm::vec3>& positions, vector<glm::vec3>& normals, vector<glm::uvec3> & indices, unsigned int strips, unsigned int slices)
{
    float radius=1.0;
    glm::vec3 center(0.0,0.0,0.0);

    unsigned int vertexNumber = slices*(strips-2) + 2;
    positions.resize(vertexNumber);
    normals.resize(vertexNumber);

    unsigned int triangleNumber = slices * (strips * 2 - 2);
    indices.resize(triangleNumber);

    positions[0] = glm::vec3(0, 0, 1);
    positions[1] = glm::vec3(0, 0, -1);
    normals[0] = glm::vec3(0, 0, 1);
    normals[1] = glm::vec3(0, 0, -1);

    double theta_step = 2.0*M_PI/(double)slices;
    double phi_step = M_PI/(double)(strips-1);
    double phi = phi_step;
    double theta = 0;

    unsigned int reduced_strips = strips - 2;

    size_t start_easy_tindex = 2 * slices * reduced_strips;

    for(int i=0; i<slices; ++i,
        phi=phi_step, theta+=theta_step)
    {
        size_t next_i = (i + 1) % slices;
        for(int j=0; j<reduced_strips; ++j, 
            phi+=phi_step)
        {
            size_t vindex = i * reduced_strips + j + 2;
            size_t vindex_next = next_i * reduced_strips + j + 2;
            int delta_j = j+1;
            size_t vindex_next_side = next_i * reduced_strips + delta_j + 2;
            size_t vindex_side = i * reduced_strips + delta_j + 2;
            glm::vec3 normal = glm::vec3(cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi));
            positions[vindex] = radius * normal;
            normals[vindex] = normal;   
            if (j < reduced_strips - 1){
                indices[2 * (i * reduced_strips + j) + 0] = glm::uvec3(vindex, vindex_next, vindex_side);
                indices[2 * (i * reduced_strips + j) + 1] = glm::uvec3(vindex_next, vindex_next_side, vindex_side);
            }
          
        }
        size_t first_vindex = i * reduced_strips + 2;
        size_t last_vindex = first_vindex + reduced_strips - 1;
        size_t first_vindex_next = next_i * reduced_strips + 2;
        size_t last_vindex_next = first_vindex_next + reduced_strips -1;
        indices[start_easy_tindex + 2 * i + 0] = glm::uvec3(0, first_vindex, first_vindex_next);
        indices[start_easy_tindex + 2 * i + 1] = glm::uvec3(1, last_vindex, last_vindex_next);
    }
}

void getUnitCone(vector<glm::vec3>& positions, vector<glm::vec3>& normals, std::vector<glm::vec2> & tcoords, unsigned int slices, bool vertex_normals)
{

    size_t number_of_triangles = slices * 2;
    size_t number_of_vertices = number_of_triangles * 3;
    float angle_step = 2.0 * M_PI / double( slices );

    positions.resize(number_of_vertices, glm::vec3(0.0,0.0,0.0));
    normals.resize(number_of_vertices, glm::vec3(0.0,0.0,0.0));
    tcoords.resize(number_of_vertices, glm::vec2(0,0));

    float previous_angle = (slices - 1) * angle_step;
    float angle = 0;
    float previous_sin = std::sin( previous_angle );
    float previous_cos = std::cos( previous_angle );
    float sin = 0;
    float cos = 1;
    glm::vec3 tip(0,0,1);
    glm::vec3 base_center(0,0,0);

    for( size_t i = 0; i < slices;
            ++ i,
            previous_sin = sin, previous_cos = cos,
            angle += angle_step, cos = std::cos( angle ), sin = std::sin( angle) )
    {
        size_t voffset = 6 * i;
        glm::vec3 p1(previous_cos, previous_sin, 0);
        glm::vec3 p2(cos, sin, 0);

        positions[ voffset +  0 ] = tip;
        positions[ voffset +  1 ] = p1;
        positions[ voffset +  2 ] = p2;

        positions[ voffset +  3 ] = p2;
        positions[ voffset +  4 ] = p1;
        positions[ voffset +  5 ] = base_center;

        if (vertex_normals){
            glm::vec3 to_tip1 = glm::normalize(tip - p1);
            glm::vec3 outward1 = glm::cross(p1, to_tip1);
            glm::vec3 normal1 = glm::cross(outward1, to_tip1);
            if (glm::dot(normal1, p1) < 0)
                normal1 *= -1;
            glm::vec3 to_tip2 = glm::normalize(tip - p2);
            glm::vec3 outward2 = glm::cross(p2, to_tip2);
            glm::vec3 normal2 = glm::cross(outward2, to_tip2);
            if (glm::dot(normal2, p2) < 0)
                normal2 *= -1;
            normals[ voffset +  0 ] = glm::normalize(normal1 + normal2);
            normals[ voffset +  1 ] = normal1;
            normals[ voffset +  2 ] = normal2;
        }else{
            glm::vec3 contour_edge = p2 - p1; 
            glm::vec3 depth_edge = tip - p1;
            glm::vec3 side_normal = glm::cross(glm::normalize(contour_edge), glm::normalize(depth_edge));

            if (glm::dot(side_normal, glm::normalize(p1+p2)) < 0)
                side_normal *= -1;

            normals[ voffset +  0 ] = side_normal;
            normals[ voffset +  1 ] = side_normal;
            normals[ voffset +  2 ] = side_normal;
        }

        normals[ voffset +  3 ] = glm::vec3{0,0,-1}; 
        normals[ voffset +  4 ] = glm::vec3{0,0,-1}; 
        normals[ voffset +  5 ] = glm::vec3{0,0,-1}; 

        float u = (i+1) / (float)slices;
        float previous_u = i / (float)slices;
        tcoords[ voffset + 0 ] = glm::vec2(previous_u, 1);
        tcoords[ voffset + 1 ] = glm::vec2(previous_u, 0.5);
        tcoords[ voffset + 2 ] = glm::vec2(u, 0.5);

        tcoords[ voffset + 3 ] = glm::vec2(u, 0.5);
        tcoords[ voffset + 4 ] = glm::vec2(previous_u, 0.5);
        tcoords[ voffset + 5 ] = glm::vec2(previous_u, 0);
    }
}
void getUnitIndexedCone(vector<glm::vec3>& positions, vector<glm::vec3>& normals, std::vector<glm::vec2> & tcoords, vector<glm::uvec3> & indices, unsigned int slices)
{
    size_t number_of_triangles = slices * 2;
    size_t number_of_vertices = 2 + slices;

    float angle_step = 2.0 * M_PI / double( slices );

    positions.resize(number_of_vertices, glm::vec3(0.0,0.0,0.0));
    normals.resize(number_of_vertices, glm::vec3(0.0,0.0,0.0));
    tcoords.resize(number_of_vertices, glm::vec2(0.0,0.0));
    indices.resize(number_of_triangles, glm::uvec3(0));

    float angle = 0;
    float sin = 0;
    float cos = 1;
    glm::vec3 tip(0,0,1);
    glm::vec3 base_center(0,0,0);
    positions[0] = base_center;
    positions[1] = tip;
    normals[0] = glm::vec3(0,0,-1);
    normals[1] = glm::vec3(0,0,1);
    tcoords[0] = glm::vec2(0.5,0);
    tcoords[1] = glm::vec2(0.5,1);

    for( size_t i = 0; i < slices;
            ++ i,
            angle += angle_step, cos = std::cos( angle ), sin = std::sin( angle) )
    {
        cout<<i<<endl;
        size_t vindex = i + 2;
        glm::vec3 p = glm::vec3(cos, sin, 0);
        positions[vindex] = p;

        glm::vec3 to_tip = glm::normalize(tip - p);
        glm::vec3 outward = glm::cross(p, to_tip);
        glm::vec3 normal = glm::cross(outward, to_tip);

        if (glm::dot(normal, p) < 0)
            normal *= -1;

        normals[vindex] = normal;

        size_t tindex = 2 * i;
        size_t next_i = (i + 1) % slices;
        size_t next_vindex = next_i + 2;
        indices[tindex + 0] = glm::uvec3(vindex, next_vindex, 0);
        indices[tindex + 1] = glm::uvec3(vindex, next_vindex, 1);

        tcoords[vindex] = glm::vec2(i/(float)slices, 0.5);
    }
}

void getUnitTorus(vector<glm::vec3>& positions, vector<glm::vec3>& normals, vector<glm::vec2>& tcoords, float tube, unsigned int strips, unsigned int slices, bool vertex_normals)
{
    vector<glm::uvec3> indices;
    vector<glm::vec3> indexed_positions;
    vector<glm::vec3> indexed_normals;
    getUnitIndexedTorus(indexed_positions, indexed_normals, indices, tube, strips, slices);
    deindex(indexed_positions, indices, positions);
    deindex(indexed_normals, indices, normals);
    if (!vertex_normals){
        vector<glm::vec3> tmp_normals = normals;
        for (size_t i=0; i < tmp_normals.size()/3 ; ++i){
            glm::vec3 mean_normal = glm::normalize(tmp_normals[3 * i + 0] + tmp_normals[3 * i + 1] + tmp_normals[3 * i + 2]);
            normals[3 * i + 0] = mean_normal;
            normals[3 * i + 1] = mean_normal;
            normals[3 * i + 2] = mean_normal;
        }
    }
    tcoords.resize(positions.size());
    for ( int j = 0; j < slices; j ++ ) {
        float v = (j+1) / (float)slices;
        float previous_v = j / (float)slices;
        for ( int i = 0; j < strips; i ++ ) {
            float u = (i+1) / (float)strips;
            float previous_u = i / (float)strips;
            size_t voffset =  6 * (j * slices + i);
            tcoords[voffset + 0] = glm::vec2(u, v);
            tcoords[voffset + 1] = glm::vec2(u, previous_v);
            tcoords[voffset + 2] = glm::vec2(previous_u, previous_v);
            tcoords[voffset + 3] = glm::vec2(previous_u, v);
            tcoords[voffset + 4] = glm::vec2(u, v);
            tcoords[voffset + 5] = glm::vec2(previous_u, previous_v);
        }
    }
}

void getUnitIndexedTorus(vector<glm::vec3>& positions, vector<glm::vec3>& normals, vector<glm::uvec3> & indices, float tube, unsigned int strips, unsigned int slices)
{
    positions.resize(0);
    normals.resize(0);
    indices.resize(0);

    for ( int j = 0; j < slices; j ++ ) {
        float v = j / float(slices);
        float v_angle = v * M_PI * 2;
        float cosv = cos(v_angle);
        float sinv = sin(v_angle);
        for ( int i = 0; i < strips; i ++ ) {
            float u = i / float(strips);
            float u_angle = u * M_PI * 2;
            float cosu = cos(u_angle);
            float sinu = sin(u_angle);

            glm::vec3 point = {(1 + tube * cosv) * cosu, (1 + tube * cosv) * sinu, tube * sinv};
            positions.push_back(point);
            glm::vec3 center = glm::vec3(cosu, sinu, 0);
            normals.push_back(glm::normalize(point - center));

            unsigned int next_j = (j+1) % slices;
            unsigned int next_i = (i+1) % strips;
            unsigned int a =  strips * next_j + i;
            unsigned int b =  strips * j + i;
            unsigned int c =  strips * j + next_i;
            unsigned int d =  strips * next_j + next_i;
            indices.push_back(glm::uvec3(a, b, d));
            indices.push_back(glm::uvec3(b, c, d));
        }
    }
}


glm::mat4 getTranslationMatrix(const glm::vec3 & tvec){
    return glm::translate(glm::mat4(), tvec);
}

glm::mat4 getTranslationMatrix(float x, float y, float z){
    return getTranslationMatrix(glm::vec3(x, y, z));
}

glm::mat4 getTranslationMatrix(float x){
    return getTranslationMatrix(glm::vec3(x));
}

glm::mat4 getScaleMatrix(const glm::vec3 & svec){
    return glm::scale(glm::mat4(), svec);
}

glm::mat4 getScaleMatrix(float x, float y, float z){
    return getScaleMatrix(glm::vec3(x, y, z));
}

glm::mat4 getScaleMatrix(float x){
    return getScaleMatrix(glm::vec3(x));
}

glm::mat4 getRotationMatrix(float angle, const glm::vec3 & avec){
    return glm::rotate(glm::mat4(), angle, glm::normalize(avec));
}

glm::mat4 getRotationMatrix(float angle, float x, float y, float z){
    return getRotationMatrix(angle, glm::vec3(x, y, z));
}

glm::mat4 lookAtModel(const glm::vec3 & position, const glm::vec3 & target, const glm::vec3 & forward)
{
    glm::vec3 dir = glm::normalize(target - position);
    glm::quat orientation = glm::rotation(forward, dir);
    return getTranslationMatrix(position) * glm::toMat4(orientation);
}

glm::mat4 lookAtUp(const glm::vec3 & position, const glm::vec3 & target)
{
    return glm::lookAt(position, target, Camera::base_up);
}

glm::mat4 lookAtUpModel(const glm::vec3 & position, const glm::vec3 & target)
{
    return glm::inverse(lookAtUp(position, target));
}

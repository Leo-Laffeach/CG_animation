#include "./../include/SphereMeshRenderable.hpp"
#include "./../include/gl_helper.hpp"
#include "./../include/log.hpp"
#include "./../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>

SphereMeshRenderable::~SphereMeshRenderable()
{}

SphereMeshRenderable::SphereMeshRenderable(ShaderProgramPtr shaderProgram, bool indexed, unsigned int strips, unsigned int slices, bool vertex_normals) :
    MeshRenderable(shaderProgram, indexed)
{
    if (indexed){
        std::vector<glm::uvec3> indices;
        getUnitIndexedSphere(m_positions, m_normals, indices, strips, slices);
        // getUniIndexedCube fills a std::vector<glm::uvec3> of length n,
        // but m_indices is a std::vector<unsigned int> of length 3n.
        // We need to unpack the values.
        unpack(indices, m_indices);
        // Set random colors per vertex
        m_colors.resize(m_positions.size(), glm::vec4(0));
        for (size_t i=0; i<m_colors.size(); ++i)
            m_colors[i] = randomColor();
    }else{
        // Go to Utils.cpp and fill this function
        getUnitSphere(m_positions, m_normals, m_tcoords, strips, slices, vertex_normals);
        // Set random colors per triangle
        m_colors.resize(m_positions.size(), glm::vec4(0));
        for (size_t i=0; i<m_colors.size() / 3; ++i){
            glm::vec4 color = randomColor();
            m_colors[ 3 * i + 0 ] = color;
            m_colors[ 3 * i + 1 ] = color;
            m_colors[ 3 * i + 2 ] = color;
        }
    }

    // See MeshRenderable::update_all_buffers
    update_all_buffers();
}
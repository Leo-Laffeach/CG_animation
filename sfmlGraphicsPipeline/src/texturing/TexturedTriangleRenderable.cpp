#include "./../../include/texturing/TexturedTriangleRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <SFML/Graphics/Image.hpp>
#include <iostream>

TexturedTriangleRenderable::~TexturedTriangleRenderable()
{}

TexturedTriangleRenderable::TexturedTriangleRenderable(ShaderProgramPtr shaderProgram, const std::string& filename)
    : TexturedMeshRenderable(shaderProgram, false)
{
    //Initialize geometry

    m_positions.push_back(glm::vec3(-1.0, 0.0, 0.0));
    m_positions.push_back(glm::vec3( 1.0, 0.0, 0.0));
    m_positions.push_back(glm::vec3( 0.0, 1.0, 0.0));

    m_normals.push_back(glm::vec3( 0.0, 0.0, 1.0));
    m_normals.push_back(glm::vec3( 0.0, 0.0, 1.0));
    m_normals.push_back(glm::vec3( 0.0, 0.0, 1.0));

    m_original_tcoords.push_back(glm::vec2(0.2, 0.2));
    m_original_tcoords.push_back(glm::vec2(1.0, 0.0));
    m_original_tcoords.push_back(glm::vec2(0.5, 1.0));

    m_tcoords = m_original_tcoords;
    m_colors.resize(m_positions.size(), glm::vec4(1.0,1.0,1.0,1.0));

    // Load texture
    m_image.loadFromFile(filename);
    m_image.flipVertically(); // sfml inverts the v axis... put the image in OpenGL convention: lower left corner is (0,0)

    // Update all buffers
    update_all_buffers();
}

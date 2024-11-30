#include "./../../include/texturing/TexturedPlaneRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <SFML/Graphics/Image.hpp>
#include <iostream>

TexturedPlaneRenderable::~TexturedPlaneRenderable()
{}

TexturedPlaneRenderable::TexturedPlaneRenderable(ShaderProgramPtr shaderProgram, const std::string& filename)
    : TexturedMeshRenderable(shaderProgram, false)
{
    // Initialize geometry
    getUnitPlane(m_positions, m_normals, m_original_tcoords);
    m_tcoords = m_original_tcoords;
    m_colors.resize(m_positions.size(), glm::vec4(1.0,1.0,1.0,1.0));

    // Load texture
    m_image.loadFromFile(filename);
    m_image.flipVertically(); // sfml inverts the v axis... put the image in OpenGL convention: lower left corner is (0,0)
    
    // Update the all buffers
    update_all_buffers();
}
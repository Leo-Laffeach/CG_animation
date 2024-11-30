#include "./../../include/texturing/TexturedCubeRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <SFML/Graphics/Image.hpp>
#include <iostream>

TexturedCubeRenderable::~TexturedCubeRenderable()
{}

TexturedCubeRenderable::TexturedCubeRenderable(ShaderProgramPtr shaderProgram, const std::string& filename)
    : TexturedMeshRenderable(shaderProgram, false)
{
    //Initialize geometry
    getUnitCube(m_positions, m_normals, m_tcoords);
    m_original_tcoords = m_tcoords;
    m_colors.resize(m_positions.size(), glm::vec4(1.0,1.0,1.0,1.0));

    // Load image
    m_image.loadFromFile(filename);
    m_image.flipVertically(); // sfml inverts the v axis... put the image in OpenGL convention: lower left corner is (0,0)
    
    update_all_buffers();
}
#include "./../../include/texturing/TexturedLightedMeshRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Io.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

TexturedLightedMeshRenderable::~TexturedLightedMeshRenderable()
{
}

TexturedLightedMeshRenderable::TexturedLightedMeshRenderable(
    ShaderProgramPtr program,
    const std::string & mesh_filename,
    const MaterialPtr & material,
    const std::string & texture_filename) :
    TexturedMeshRenderable(program, mesh_filename, texture_filename),
    m_material(material)
{}

TexturedLightedMeshRenderable::TexturedLightedMeshRenderable(
    ShaderProgramPtr program,
    const std::vector< glm::vec3 > & positions,
    const std::vector< unsigned int > & indices,
    const std::vector< glm::vec3 > & normals,
    const std::vector< glm::vec4 > & colors,
    const MaterialPtr & material,
    const sf::Image & image,
    const std::vector< glm::vec2 > & tcoords) :
    TexturedMeshRenderable(program, positions, indices, normals, colors, image, tcoords),
    m_material(material)
{}

TexturedLightedMeshRenderable::TexturedLightedMeshRenderable(
    ShaderProgramPtr program,
    const std::vector< glm::vec3 > & positions,
    const std::vector< glm::vec3 > & normals,
    const std::vector< glm::vec4 > & colors,
    const MaterialPtr & material,
    const sf::Image & image,
    const std::vector< glm::vec2 > & tcoords) :
    TexturedMeshRenderable(program, positions, normals, colors, image, tcoords),
    m_material(material)
{}


void TexturedLightedMeshRenderable::do_draw()
{
    //Send material uniform to GPU
    Material::sendToGPU(m_shaderProgram, m_material);
    TexturedMeshRenderable::do_draw();
}

const MaterialPtr & TexturedLightedMeshRenderable::getMaterial() const
{
    return m_material;
}

void TexturedLightedMeshRenderable::setMaterial(const MaterialPtr & mat)
{
    m_material = mat;
}

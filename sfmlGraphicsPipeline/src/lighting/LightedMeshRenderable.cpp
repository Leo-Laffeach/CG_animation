#include "./../../include/lighting/LightedMeshRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Io.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

LightedMeshRenderable::~LightedMeshRenderable()
{}

LightedMeshRenderable::LightedMeshRenderable(ShaderProgramPtr shaderProgram,
                                             const std::vector< glm::vec3 > & positions,
                                             const std::vector< unsigned int > & indices,
                                             const std::vector< glm::vec3 > & normals,
                                             const std::vector< glm::vec4 > & colors,
                                             const MaterialPtr & material) :
    MeshRenderable(shaderProgram, positions, indices, normals, colors),
    m_material(m_material)
{}

LightedMeshRenderable::LightedMeshRenderable(ShaderProgramPtr shaderProgram,
                                             const std::vector< glm::vec3 > & positions,
                                             const std::vector< glm::vec3 > & normals,
                                             const std::vector< glm::vec4 > & colors,
                                             const MaterialPtr & material) :
    MeshRenderable(shaderProgram, positions, normals, colors),
    m_material(material)
{}

LightedMeshRenderable::LightedMeshRenderable(ShaderProgramPtr shaderProgram, 
                                             const std::string & mesh_filename,
                                             const MaterialPtr & material) :
    MeshRenderable(shaderProgram, mesh_filename),
    m_material(material)
{}

LightedMeshRenderable::LightedMeshRenderable(ShaderProgramPtr shaderProgram, bool indexed, const MaterialPtr & material):
    MeshRenderable(shaderProgram, indexed), m_material(material)
{}

void LightedMeshRenderable::do_draw()
{
    //Send material uniform to GPU
    Material::sendToGPU(m_shaderProgram, m_material);
    MeshRenderable::do_draw();
}

const MaterialPtr & LightedMeshRenderable::getMaterial() const
{
    return m_material;
}

void LightedMeshRenderable::setMaterial(const MaterialPtr & mat)
{
    m_material = mat;
}

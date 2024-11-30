#include "./../../include/lighting/PointLightRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>

PointLightRenderable::PointLightRenderable(const ShaderProgramPtr & shaderProgram, const PointLightPtr & light, unsigned int strips, unsigned int slices) :
    MeshRenderable(shaderProgram, true), m_light(light)
{
    std::vector<glm::uvec3> uvec3_indices;
    getUnitIndexedSphere(m_positions, m_normals, uvec3_indices, strips, slices);
    unpack(uvec3_indices, m_indices);
    m_colors.resize(m_positions.size(), glm::vec4(light->diffuse(),1.0));

    glm::mat4 transformation = getTranslationMatrix(m_light->position());
    setGlobalTransform(transformation);

    update_all_buffers();
}

void PointLightRenderable::do_animate(float time){
    setGlobalTransform(m_light->getGlobalTransform());
}

PointLightRenderable::~PointLightRenderable()
{}

#include "./../../include/lighting/SpotLightRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>

SpotLightRenderable::SpotLightRenderable(const ShaderProgramPtr & prog, const SpotLightPtr & light, unsigned int slices) :
    MeshRenderable(prog, false), m_light(light)
{
    getUnitCone(m_positions, m_normals, m_tcoords, slices, true);
    
    m_colors.resize(m_positions.size(), glm::vec4(light->diffuse(),1.0));

    // Transform according to m_light
    setGlobalTransform(m_light->getGlobalTransform());

    //Scale according to the spot angle
    float scaleFactor = std::tan(std::acos(m_light->outerCutOff()))*2.0f;
    glm::mat4 scale = getScaleMatrix(scaleFactor, scaleFactor, 1);
    setLocalTransform(scale);

    update_all_buffers();
}

void SpotLightRenderable::do_animate(float time)
{
    setGlobalTransform(m_light->getGlobalTransform());
}

SpotLightRenderable::~SpotLightRenderable()
{}

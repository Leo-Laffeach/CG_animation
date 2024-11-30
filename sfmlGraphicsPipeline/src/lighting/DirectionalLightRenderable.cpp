#include "./../../include/lighting/DirectionalLightRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>

DirectionalLightRenderable::DirectionalLightRenderable(ShaderProgramPtr shaderProgram, DirectionalLightPtr light) :
    MeshRenderable(shaderProgram, false), m_light(light)
{  
    // Cone 
    std::vector<glm::vec3> tmp_x, tmp_n;
    unsigned int strips=20, slices=20;
    glm::mat4 transformation(1.0);
    transformation = getTranslationMatrix(0,0,1) * getScaleMatrix(1,1,-1);
    getUnitCone(tmp_x, tmp_n, m_tcoords, slices, true);
    for(size_t i=0; i<tmp_x.size(); ++i) m_positions.push_back(glm::vec3(transformation*glm::vec4(tmp_x[i],1.0)));
    m_normals.insert(m_normals.end(), tmp_n.begin(), tmp_n.end());

    //Cylinder
    transformation = getTranslationMatrix(0,0,1) * getScaleMatrix(0.5,0.5,2.0);
    getUnitCylinder(tmp_x, tmp_n, m_tcoords, strips, true);
    for(size_t i=0; i<tmp_x.size(); ++i) m_positions.push_back(glm::vec3(transformation*glm::vec4(tmp_x[i],1.0)));
    m_normals.insert(m_normals.end(), tmp_n.begin(), tmp_n.end());

    // Color
    m_colors.resize(m_positions.size(), glm::vec4(light->diffuse(),1.0));

    // Transforms
    setGlobalTransform(light->getGlobalTransform());
    // No need to call updateModelMatrix() here
    // because it will be called in Renderable::beforeDraw in Renderable::draw

    update_all_buffers();
}

void DirectionalLightRenderable::do_animate(float time){
    setGlobalTransform(m_light->getGlobalTransform());
    // No need to call updateModelMatrix() here
    // because it will be called in Renderable::beforeDraw in Renderable::draw
}

DirectionalLightRenderable::~DirectionalLightRenderable()
{}
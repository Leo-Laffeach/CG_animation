#include "../../include/dynamics/SpringForceFieldRenderable.hpp"
#include "../../include/log.hpp"
#include "../../include/gl_helper.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

SpringForceFieldRenderable::~SpringForceFieldRenderable()
{}

SpringForceFieldRenderable::SpringForceFieldRenderable(ShaderProgramPtr shaderProgram, SpringForceFieldPtr springForceField) :
    MeshRenderable(shaderProgram, false),
    m_springForceField(springForceField)
{
    m_mode = GL_LINES;
    //Create geometric data
    m_positions = {m_springForceField->getParticle1()->getPosition(),
                   m_springForceField->getParticle2()->getPosition()};
    m_colors.resize(2, glm::vec4(0,0,1,1));
    m_normals.resize(2, glm::vec3(1,0,0));

    update_all_buffers();
}

void SpringForceFieldRenderable::do_draw()
{
    glLineWidth(3.0);
    MeshRenderable::do_draw();
    glLineWidth(1.0);
}
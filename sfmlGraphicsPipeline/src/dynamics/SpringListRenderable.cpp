#include "./../../include/dynamics/SpringListRenderable.hpp"
#include "../../include/log.hpp"
#include "../../include/gl_helper.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

SpringListRenderable::~SpringListRenderable()
{}

SpringListRenderable::SpringListRenderable(ShaderProgramPtr shaderProgram, std::list<SpringForceFieldPtr>& springForceFields) :
    MeshRenderable(shaderProgram, false),
    m_springForceFields(springForceFields)
{
    m_mode = GL_LINES;
    //Create geometric data
    size_t springNumber =  m_springForceFields.size();
    m_positions.resize(2*springNumber);
    m_colors.resize(2*springNumber, glm::vec4(0.0,0.0,1.0,1.0));
    m_normals.resize(2*springNumber, glm::vec3(1.0,1.0,1.0));

    update_spring_positions();

    update_all_buffers();
}

void SpringListRenderable::update_spring_positions(){
    size_t i = 0;
    for (const SpringForceFieldPtr & spring : m_springForceFields){
        m_positions[2*i+0] = spring->getParticle1()->getPosition();
        m_positions[2*i+1] = spring->getParticle2()->getPosition();
        ++i;
    }
}

void SpringListRenderable::do_draw()
{
    //Update vertices positions from particle's positions
    update_spring_positions();
    update_positions_buffer();

    glLineWidth(3.0);
    MeshRenderable::do_draw();
    glLineWidth(1.0);
}

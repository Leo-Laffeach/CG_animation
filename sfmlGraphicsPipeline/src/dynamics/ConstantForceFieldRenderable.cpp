#include "../../include/dynamics/ConstantForceFieldRenderable.hpp"
#include "../../include/log.hpp"
#include "../../include/gl_helper.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

ConstantForceFieldRenderable::~ConstantForceFieldRenderable()
{}

ConstantForceFieldRenderable::ConstantForceFieldRenderable(ShaderProgramPtr shaderProgram, ConstantForceFieldPtr forceField) :
    MeshRenderable(shaderProgram, false),
    m_forceField(forceField)
{
    //Create geometric data
    const std::vector<ParticlePtr> & particles = m_forceField->getParticles();
    m_positions.resize(2*particles.size());
    m_colors.resize(2*particles.size());
    m_normals.resize(2*particles.size()); 
    update_particle_positions();
    // Render as lines
    m_mode = GL_LINES;
}

void ConstantForceFieldRenderable::update_particle_positions()
{
    const std::vector<ParticlePtr> & particles = m_forceField->getParticles();
    for(size_t i=0;i<particles.size();++i)
    {
        m_positions[2*i+0] = particles[i]->getPosition();
        m_positions[2*i+1] = particles[i]->getPosition() + 0.1f*m_forceField->getForce();
        m_colors[2*i+0] = glm::vec4(1.0,0.0,0.0,1.0);
        m_colors[2*i+1] = glm::vec4(1.0,0.0,0.0,1.0);
        m_normals[2*i+0] = glm::vec3(1.0,0.0,0.0);
        m_normals[2*i+1] = glm::vec3(1.0,0.0,0.0);
    }
    update_all_buffers();
}

void ConstantForceFieldRenderable::do_draw()
{
    //Update vertices positions from particle's positions
    update_particle_positions();
    // Draw them
    glLineWidth(3.0);
    MeshRenderable::do_draw();
    glLineWidth(1.0);
}
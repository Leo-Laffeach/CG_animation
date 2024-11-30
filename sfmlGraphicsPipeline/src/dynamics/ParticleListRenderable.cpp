/* Created on: Mar 7, 2016
 *     Author: T.Delame (tdelame@gmail.com)
 */
#include "../../include/dynamics/ParticleListRenderable.hpp"
#include <glm/gtc/type_ptr.hpp>

ParticleListRenderable::~ParticleListRenderable()
{
    glcheck(glDeleteBuffers(1, &m_pBuffer));
    glcheck(glDeleteBuffers(1, &m_cBuffer));
    glcheck(glDeleteBuffers(1, &m_nBuffer));
    glcheck(glDeleteBuffers(1, &m_iBuffer));
    glcheck(glDeleteBuffers(1, &m_idBuffer));
}

ParticleListRenderable::ParticleListRenderable(ShaderProgramPtr program, std::vector<ParticlePtr>& particles, unsigned int strips, unsigned int slices) :
    HierarchicalRenderable( program ),
    m_particles(particles), m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_idBuffer(0)
{   
    std::vector<glm::uvec3> uvec3_indices;
    getUnitIndexedSphere( m_positions, m_normals, uvec3_indices, strips, slices);
    // Need to unpack the indices
    unpack(uvec3_indices, m_indices),

    m_colors.resize( m_positions.size() );
    for( size_t i = 0; i < m_positions.size(); ++ i )
        m_colors[i] = getColor( m_positions[i].x, -1, 1 );

    genbuffers();
    update_all_buffers();
}

// The implementation does not follow MeshRenderable::do_draw as usual
// because we are doing instanced rendering !
void ParticleListRenderable::do_draw()
{  
    update_instances_data_buffer();
    int positionLocation = m_shaderProgram->getAttributeLocation("vPosition");
    int colorLocation = m_shaderProgram->getAttributeLocation("vColor");
    int normalLocation = m_shaderProgram->getAttributeLocation("vNormal");
    int modelLocation = m_shaderProgram->getUniformLocation("modelMat");
    int nitLocation = m_shaderProgram->getUniformLocation("NIT");
    int instanceDataLocation = m_shaderProgram->getAttributeLocation("instanceData");
    
    if(modelLocation != ShaderProgram::null_location)
        glcheck(glUniformMatrix4fv(modelLocation, 1, GL_FALSE, glm::value_ptr(getModelMatrix())));

    if(positionLocation != ShaderProgram::null_location)
    {
        glcheck(glEnableVertexAttribArray(positionLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_pBuffer));
        glcheck(glVertexAttribPointer(positionLocation, 3, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    if(colorLocation != ShaderProgram::null_location)
    {
        glcheck(glEnableVertexAttribArray(colorLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_cBuffer));
        glcheck(glVertexAttribPointer(colorLocation, 4, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    if(normalLocation != ShaderProgram::null_location)
    {
        glcheck(glEnableVertexAttribArray(normalLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_nBuffer));
        glcheck(glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    if( nitLocation != ShaderProgram::null_location )
    {
        glcheck(glUniformMatrix3fv( nitLocation, 1, GL_FALSE,
        glm::value_ptr(glm::transpose(glm::inverse(glm::mat3(getModelMatrix()))))));
    }

    if ( instanceDataLocation != ShaderProgram::null_location )
    {
        glcheck(glEnableVertexAttribArray(instanceDataLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_idBuffer));
        glcheck(glVertexAttribPointer(instanceDataLocation, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), (void*)0));
        glVertexAttribDivisor(instanceDataLocation, 1);
    }

    // Draw instanced triangles elements
    glcheck(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iBuffer));
    glcheck(glDrawElementsInstanced(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0, m_particles.size()));
    glcheck(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    if(positionLocation != ShaderProgram::null_location)
    {
        glcheck(glDisableVertexAttribArray(positionLocation));
    }

    if(colorLocation != ShaderProgram::null_location)
    {
        glcheck(glDisableVertexAttribArray(colorLocation));
    }

    if(normalLocation != ShaderProgram::null_location)
    {
        glcheck(glDisableVertexAttribArray(normalLocation));
    }

    if(instanceDataLocation != ShaderProgram::null_location)
    {
        glcheck(glDisableVertexAttribArray(instanceDataLocation));
        glVertexAttribDivisor(instanceDataLocation, 0);
    }
}

void ParticleListRenderable::genbuffers(){
    glGenBuffers(1, &m_pBuffer); //vertices
    glGenBuffers(1, &m_cBuffer); //colors
    glGenBuffers(1, &m_nBuffer); //normals
    glGenBuffers(1, &m_iBuffer); //indices
    glGenBuffers(1, &m_idBuffer); //instance data
}

void ParticleListRenderable::update_all_buffers(){
    update_positions_buffer();
    update_normals_buffer();
    update_colors_buffer();
    update_indices_buffer();
    update_instances_data_buffer();
}

void ParticleListRenderable::update_positions_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_pBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_positions.size()*sizeof(glm::vec3), m_positions.data(), GL_STATIC_DRAW));
}

void ParticleListRenderable::update_normals_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_pBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_normals.size()*sizeof(glm::vec3), m_normals.data(), GL_STATIC_DRAW));
}

void ParticleListRenderable::update_colors_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_cBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_colors.size()*sizeof(glm::vec4), m_colors.data(), GL_STATIC_DRAW));
}

void ParticleListRenderable::update_indices_buffer(){
    glcheck(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iBuffer));
    glcheck(glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size()*sizeof(unsigned int), m_indices.data(), GL_STATIC_DRAW));
}

void ParticleListRenderable::update_instances_data_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_idBuffer));
    std::vector<glm::vec4> instances_data(m_particles.size());
    for (std::size_t i=0u; i<m_particles.size(); ++i)
        instances_data[i] = glm::vec4(m_particles[i]->getPosition(), m_particles[i]->getRadius());
    glcheck(glBufferData(GL_ARRAY_BUFFER, instances_data.size()*sizeof(glm::vec4), instances_data.data(), GL_STREAM_DRAW));
}
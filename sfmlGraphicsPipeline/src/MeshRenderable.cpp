#include "./../include/MeshRenderable.hpp"
#include "./../include/gl_helper.hpp"
#include "./../include/log.hpp"
#include "./../include/Io.hpp"
#include "./../include/Utils.hpp"


#include <glm/gtc/type_ptr.hpp>


MeshRenderable::MeshRenderable(ShaderProgramPtr program,
                               const std::string & mesh_filename) :
    KeyframedHierarchicalRenderable(program),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES), m_indexed(true)
{
    // use read_obj from Io.hpp to populate m_positions, m_indices, m_normals and m_tcoords
    read_obj(mesh_filename, m_positions, m_indices, m_normals, m_tcoords);
    set_random_colors();
    gen_buffers();
    update_buffers();
}

MeshRenderable::MeshRenderable(ShaderProgramPtr program,
                               const std::vector< glm::vec3 > & positions,
                               const std::vector< unsigned int > & indices, 
                               const std::vector< glm::vec3 > & normals,
                               const std::vector< glm::vec4 > & colors) :
    KeyframedHierarchicalRenderable(program),
    m_positions(positions), m_indices(indices), m_normals(normals), m_colors(colors),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES), m_indexed(true)
{
    set_random_colors();
    gen_buffers();
    update_buffers();
}

MeshRenderable::MeshRenderable(ShaderProgramPtr program,
                               const std::vector< glm::vec3 > & positions,
                               const std::vector< glm::vec3 > & normals,
                               const std::vector< glm::vec4 > & colors) :
    KeyframedHierarchicalRenderable(program),
    m_positions(positions), m_normals(normals), m_colors(colors),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES), m_indexed(false)
{
    set_random_colors();
    gen_buffers();
    update_buffers();
}

MeshRenderable::MeshRenderable(ShaderProgramPtr program, bool indexed) :
    KeyframedHierarchicalRenderable(program), m_indexed(indexed),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES)
{
    gen_buffers();
}

void MeshRenderable::gen_buffers(){
    //Create buffers
    glGenBuffers(1, &m_pBuffer); //vertices
    glGenBuffers(1, &m_cBuffer); //colors
    glGenBuffers(1, &m_nBuffer); //normals
    if (m_indexed)
        glGenBuffers(1, &m_iBuffer); //indices
}

void MeshRenderable::update_buffers(){
    //Activate buffer and send data to the graphics card
    update_positions_buffer();
    update_colors_buffer();
    update_normals_buffer();
    if (m_indexed)
        update_indices_buffer();
}

void MeshRenderable::update_all_buffers(){
    update_buffers();
}

void MeshRenderable::update_positions_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_pBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_positions.size()*sizeof(glm::vec3), m_positions.data(), GL_STATIC_DRAW));
}
void MeshRenderable::update_colors_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_cBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_colors.size()*sizeof(glm::vec4), m_colors.data(), GL_STATIC_DRAW));
}
void MeshRenderable::update_normals_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_nBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_normals.size()*sizeof(glm::vec3), m_normals.data(), GL_STATIC_DRAW));
}
void MeshRenderable::update_indices_buffer(){
    glcheck(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iBuffer));
    glcheck(glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size()*sizeof(unsigned int), m_indices.data(), GL_STATIC_DRAW));
}

void MeshRenderable::do_draw()
{
    
    int positionLocation = m_shaderProgram->getAttributeLocation("vPosition");
    int colorLocation = m_shaderProgram->getAttributeLocation("vColor");
    int normalLocation = m_shaderProgram->getAttributeLocation("vNormal");
    int modelLocation = m_shaderProgram->getUniformLocation("modelMat");
    int nitLocation = m_shaderProgram->getUniformLocation("NIT");

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

    //Draw triangles elements
    if (m_indexed){
        glcheck(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iBuffer));
        glcheck(glDrawElements(m_mode, m_indices.size(), GL_UNSIGNED_INT, (void*)0));
    }else{
        glcheck(glDrawArrays(m_mode,0, m_positions.size()));
    }

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
}

void MeshRenderable::set_random_colors(){
    if (m_colors.empty()){
        m_colors.resize( m_positions.size() );
        for(size_t i=0; i<m_colors.size(); ++i)
            m_colors[i] = randomColor();
    }
}

MeshRenderable::~MeshRenderable()
{
    glcheck(glDeleteBuffers(1, &m_pBuffer));
    glcheck(glDeleteBuffers(1, &m_cBuffer));
    glcheck(glDeleteBuffers(1, &m_nBuffer));
    glcheck(glDeleteBuffers(1, &m_iBuffer));
}
/*
#include "./../include/MeshRenderable.hpp"
#include "./../include/gl_helper.hpp"
#include "./../include/log.hpp"
#include "./../include/Io.hpp"
#include "./../include/Utils.hpp"


#include <glm/gtc/type_ptr.hpp>


MeshRenderable::MeshRenderable(ShaderProgramPtr program,
                               const std::string & mesh_filename) :
    KeyframedHierarchicalRenderable(program),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES), m_indexed(true)
{
    //read_obj(...);
    set_random_colors();
    gen_buffers();
    update_buffers();
}

MeshRenderable::MeshRenderable(ShaderProgramPtr program,
                               const std::vector< glm::vec3 > & positions,
                               const std::vector< unsigned int > & indices, 
                               const std::vector< glm::vec3 > & normals,
                               const std::vector< glm::vec4 > & colors) :
    KeyframedHierarchicalRenderable(program),
    m_positions(positions), m_indices(indices), m_normals(normals), m_colors(colors),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES), m_indexed(true)
{
    set_random_colors();
    gen_buffers();
    update_buffers();
}

MeshRenderable::MeshRenderable(ShaderProgramPtr program,
                               const std::vector< glm::vec3 > & positions,
                               const std::vector< glm::vec3 > & normals,
                               const std::vector< glm::vec4 > & colors) :
    KeyframedHierarchicalRenderable(program),
    m_positions(positions), m_normals(normals), m_colors(colors),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES), m_indexed(false)
{
    set_random_colors();
    gen_buffers();
    update_buffers();
}

MeshRenderable::MeshRenderable(ShaderProgramPtr program, bool indexed) :
    KeyframedHierarchicalRenderable(program), m_indexed(indexed),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0), m_iBuffer(0), m_mode(GL_TRIANGLES)
{
    gen_buffers();
}

void MeshRenderable::gen_buffers(){
    //Create buffers
    glGenBuffers(1, &m_pBuffer); //vertices
    glGenBuffers(1, &m_cBuffer); //colors
    glGenBuffers(1, &m_nBuffer); //normals
    if (m_indexed)
        glGenBuffers(1, &m_iBuffer); //indices
}

void MeshRenderable::update_buffers(){
    //Activate buffer and send data to the graphics card
    update_positions_buffer();
    update_colors_buffer();
    update_normals_buffer();
    if (m_indexed)
        update_indices_buffer();
}

void MeshRenderable::update_all_buffers(){
    update_buffers();
}

void MeshRenderable::update_positions_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_pBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_positions.size()*sizeof(glm::vec3), m_positions.data(), GL_STATIC_DRAW));
}
void MeshRenderable::update_colors_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_cBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_colors.size()*sizeof(glm::vec4), m_colors.data(), GL_STATIC_DRAW));
}
void MeshRenderable::update_normals_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_nBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_normals.size()*sizeof(glm::vec3), m_normals.data(), GL_STATIC_DRAW));
}
void MeshRenderable::update_indices_buffer(){
    glcheck(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iBuffer));
    glcheck(glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size()*sizeof(unsigned int), m_indices.data(), GL_STATIC_DRAW));
}

void MeshRenderable::do_draw()
{
    
    int positionLocation = m_shaderProgram->getAttributeLocation("vPosition");
    int colorLocation = m_shaderProgram->getAttributeLocation("vColor");
    int normalLocation = m_shaderProgram->getAttributeLocation("vNormal");
    int modelLocation = m_shaderProgram->getUniformLocation("modelMat");
    int nitLocation = m_shaderProgram->getUniformLocation("NIT");

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

    //Draw triangles elements
    if (m_indexed){
        glcheck(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iBuffer));
        glcheck(glDrawElements(m_mode, m_indices.size(), GL_UNSIGNED_INT, (void*)0));
    }else{
        glcheck(glDrawArrays(m_mode,0, m_positions.size()));
    }

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
}

void MeshRenderable::set_random_colors(){
    if (m_colors.empty()){
        m_colors.resize( m_positions.size() );
        for(size_t i=0; i<m_colors.size(); ++i)
            m_colors[i] = randomColor();
    }
}

MeshRenderable::~MeshRenderable()
{
    glcheck(glDeleteBuffers(1, &m_pBuffer));
    glcheck(glDeleteBuffers(1, &m_cBuffer));
    glcheck(glDeleteBuffers(1, &m_nBuffer));
    glcheck(glDeleteBuffers(1, &m_iBuffer));
}*/


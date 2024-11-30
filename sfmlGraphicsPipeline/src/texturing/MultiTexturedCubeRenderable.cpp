#include "./../../include/texturing/MultiTexturedCubeRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <SFML/Graphics/Image.hpp>
#include <iostream>

MultiTexturedCubeRenderable::~MultiTexturedCubeRenderable()
{
    glcheck(glDeleteBuffers(1, &m_tBuffer));
    glcheck(glDeleteTextures(1, &m_texId1));
    glcheck(glDeleteTextures(1, &m_texId2));
}

MultiTexturedCubeRenderable::MultiTexturedCubeRenderable(ShaderProgramPtr shaderProgram, const std::string& filename1, const std::string &filename2)
    : MeshRenderable(shaderProgram, false),
      m_tBuffer(0), m_texId1(0), m_texId2(0)
{
    //Initialize geometry
    getUnitCube(m_positions, m_normals, m_tcoords);
    m_colors.resize(m_positions.size(), glm::vec4(1.0,1.0,1.0,1.0));

    // Load images
    m_image1.loadFromFile(filename1);
    m_image1.flipVertically();
    m_image2.loadFromFile(filename2);
    m_image2.flipVertically();

    gen_buffers();
    update_all_buffers();
}

void MultiTexturedCubeRenderable::update_buffers()
{
    update_tcoords_buffer();
    update_textures_buffer();
}

void MultiTexturedCubeRenderable::update_all_buffers(){
    MeshRenderable::update_all_buffers();
    update_buffers();
}

void MultiTexturedCubeRenderable::gen_buffers(){
    //Create texture
    glGenTextures(1, &m_texId1);
    glGenTextures(1, &m_texId2);
    // Create tcoords buffer
    glGenBuffers(1, &m_tBuffer);
}

void MultiTexturedCubeRenderable::update_tcoords_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_tBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_tcoords.size()*sizeof(glm::vec2), m_tcoords.data(), GL_STATIC_DRAW));
}

void MultiTexturedCubeRenderable::update_textures_buffer(){
    //Bind the texture
    glBindTexture(GL_TEXTURE_2D, m_texId1);
    //Textured options
    
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glcheck(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_image1.getSize().x, m_image1.getSize().y, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_image1.getPixelsPtr()));
    glcheck(glGenerateMipmap(GL_TEXTURE_2D));

    glBindTexture(GL_TEXTURE_2D, m_texId2);
    //Textured options
    
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glcheck(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_image2.getSize().x, m_image2.getSize().y, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_image2.getPixelsPtr()));
    glcheck(glGenerateMipmap(GL_TEXTURE_2D));
    
    //Release the texture
    glBindTexture(GL_TEXTURE_2D, 0);
}

void MultiTexturedCubeRenderable::do_draw()
{
    //Location
    int tcoordsLocation = m_shaderProgram->getAttributeLocation("vTexCoord");
    int texSampleLoc1 = m_shaderProgram->getUniformLocation("texSampler1");
    int texSampleLoc2 = m_shaderProgram->getUniformLocation("texSampler2");

    //Bind texture in Textured Unit 0
    if(tcoordsLocation != ShaderProgram::null_location)
    {
        glcheck(glEnableVertexAttribArray(tcoordsLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_tBuffer));
        glcheck(glVertexAttribPointer(tcoordsLocation, 2, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }
    if(texSampleLoc1 != ShaderProgram::null_location){
        glcheck(glActiveTexture(GL_TEXTURE0));
        glcheck(glBindTexture(GL_TEXTURE_2D, m_texId1));
        //Send "texSampler" to Textured Unit 0
        glcheck(glUniform1i(texSampleLoc1, 0));
    }
    if(texSampleLoc2 != ShaderProgram::null_location){
        glcheck(glActiveTexture(GL_TEXTURE1));
        glcheck(glBindTexture(GL_TEXTURE_2D, m_texId2));
        //Send "texSampler" to Textured Unit 1
        glcheck(glUniform1i(texSampleLoc2, 1));
    }

    MeshRenderable::do_draw();

    //Release texture
    glcheck(glBindTexture(GL_TEXTURE_2D, 0));

    // Release tcoords
    if(tcoordsLocation != ShaderProgram::null_location)
        glcheck(glDisableVertexAttribArray(tcoordsLocation));
}

#include "./../../include/texturing/MipMapCubeRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/Utils.hpp"
#include "./../../include/log.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <SFML/Graphics/Image.hpp>
#include <iostream>

static const std::array<std::string, 4> filter_option_names = {
    "GL_LINEAR_MIPMAP_LINEAR",
    "GL_LINEAR_MIPMAP_NEAREST",
    "GL_NEAREST_MIPMAP_NEAREST",
    "GL_NEAREST_MIPMAP_LINEAR"
};


MipMapCubeRenderable::~MipMapCubeRenderable()
{
    glcheck(glDeleteBuffers(1, &m_tBuffer));
    glcheck(glDeleteTextures(1, &m_texId)); // even with several subimages, there is still a single texture!
}

MipMapCubeRenderable::MipMapCubeRenderable(ShaderProgramPtr shaderProgram, const std::vector<std::string> &filenames)
    : MeshRenderable(shaderProgram, false),
      m_tBuffer(0), m_texId(0), m_mipmapOption(0)
{
    //Initialize geometry
    getUnitCube(m_positions, m_normals, m_tcoords);
    m_colors.resize(m_positions.size(), glm::vec4(1.0,1.0,1.0,1.0));

    //Load images from files
    m_images.resize(filenames.size());
    for(int i=0; i<m_images.size(); ++i){
        m_images[i].loadFromFile(filenames[i]);
        m_images[i].flipVertically(); // sfml inverts the v axis... put the image in OpenGL convention: lower left corner is (0,0)
    }
    
    gen_buffers();
    update_all_buffers(); // We also want to update MeshRenderable's buffers since we modified them
}

void MipMapCubeRenderable::gen_buffers()
{
    glcheck(glGenBuffers(1, &m_tBuffer)); //texture coordinates
    glcheck(glGenTextures(1, &m_texId));
}
void MipMapCubeRenderable::update_buffers()
{
    update_tcoords_buffer();
    update_texture_buffer();
}

void MipMapCubeRenderable::update_all_buffers(){
    MeshRenderable::update_all_buffers();
    update_buffers();
}

void MipMapCubeRenderable::update_texture_buffer(){
    //Bind the texture
    glcheck(glBindTexture(GL_TEXTURE_2D, m_texId));
    //Texture options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    // send the texture
    //Send the image to OpenGL as textures
    sf::Vector2u imageSize = m_images[0].getSize();
    glTexStorage2D(GL_TEXTURE_2D, m_images.size(), GL_RGBA32F, imageSize.x, imageSize.y);
    for(int i=0; i<m_images.size(); ++i)
    {
        glTexSubImage2D(GL_TEXTURE_2D, i, 0, 0, m_images[i].getSize().x, m_images[i].getSize().y, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_images[i].getPixelsPtr());
    }
    //Release the texture
    glcheck(glBindTexture(GL_TEXTURE_2D, 0));
}

void MipMapCubeRenderable::update_tcoords_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_tBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_tcoords.size()*sizeof(glm::vec2), m_tcoords.data(), GL_STATIC_DRAW));
}

void MipMapCubeRenderable::do_draw()
{
    //Location
    int texcoordLocation = m_shaderProgram->getAttributeLocation("vTexCoord");
    int texsamplerLocation = m_shaderProgram->getUniformLocation("texSampler");

    //Bind texture in Textured Unit 0
    if(texcoordLocation != ShaderProgram::null_location)
    {
        glcheck(glActiveTexture(GL_TEXTURE0));
        glcheck(glBindTexture(GL_TEXTURE_2D, m_texId));
        //Send "texSampler" to Textured Unit 0
        glcheck(glUniform1i(texsamplerLocation, 0));
        glcheck(glEnableVertexAttribArray(texcoordLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_tBuffer));
        glcheck(glVertexAttribPointer(texcoordLocation, 2, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    MeshRenderable::do_draw();

    glcheck(glBindTexture(GL_TEXTURE_2D, 0));
    if(texcoordLocation != ShaderProgram::null_location)
    {
        glcheck(glDisableVertexAttribArray(texcoordLocation ));
    }
}

void MipMapCubeRenderable::updateTextureOption()
{

    //Bind the texture
    glBindTexture(GL_TEXTURE_2D, m_texId);

    // Here multiple texture files are loaded
    // Otherwise, generate multiresolution images with:
    //     glcheck(glGenerateMipmap(GL_TEXTURE_2D));

    //Textured options
    switch(m_mipmapOption)
    {
    case 0:
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
        break;

    case 1:
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
        break;

    case 2:
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
        break;

    case 3:
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
        break;
    }
    LOG(info, "Texture filtering set to : "<<filter_option_names[m_mipmapOption]);

    //Release the texture
    glBindTexture(GL_TEXTURE_2D, 0);

}

void MipMapCubeRenderable::do_keyPressedEvent( sf::Event& e )
{
    if(e.key.code == sf::Keyboard::F8)
    {
        m_mipmapOption = m_mipmapOption+1 > 3 ? 0 : m_mipmapOption+1;
        updateTextureOption();
    }
}

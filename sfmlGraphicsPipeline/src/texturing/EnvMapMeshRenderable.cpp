#include "./../../include/texturing/EnvMapMeshRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/Utils.hpp"
#include "./../../include/log.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <SFML/Graphics/Image.hpp>
#include <iostream>

EnvMapMeshRenderable::~EnvMapMeshRenderable()
{
    glcheck(glDeleteTextures(1, &m_denvTexId));
    glcheck(glDeleteTextures(1, &m_senvTexId));
}

EnvMapMeshRenderable::EnvMapMeshRenderable(
    ShaderProgramPtr program,
    const std::string & mesh_filename,
    const MaterialPtr & mat,
    const std::string & texture_filename,
    const std::string & diffuse_envmap_dir,
    const std::string & specular_envmap_dir)
        : TexturedLightedMeshRenderable(program, mesh_filename, mat, texture_filename),
        m_denvTexId(0), m_senvTexId(0)
{
    // Load images
    cmutils::load_cubemap(diffuse_envmap_dir, m_diffuse_envmap);
    cmutils::load_cubemap(specular_envmap_dir, m_specular_envmap);

    // Generate and send buffers
    glGenTextures(1, &m_denvTexId);
    glGenTextures(1, &m_senvTexId);
    update_all_buffers();

    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
}

void EnvMapMeshRenderable::update_all_buffers(){
    TexturedLightedMeshRenderable::update_all_buffers();
    update_textures_buffer();
}

void EnvMapMeshRenderable::update_textures_buffer()
{
    // Diffuse cubemap
    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, m_denvTexId));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)); 

    unsigned int width = m_diffuse_envmap[0].getSize().x;
    unsigned int height = m_diffuse_envmap[0].getSize().y;

    // Send data to gpu
    for (size_t i=0u;i<m_diffuse_envmap.size();++i){
        glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_diffuse_envmap[i].getPixelsPtr()));
    }

    // Specular cubemap
    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, m_senvTexId));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)); 

    width = m_specular_envmap[0].getSize().x;
    height = m_specular_envmap[0].getSize().y;

    // Send data to gpu
    for (size_t i=0u;i<m_specular_envmap.size();++i){
        glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_specular_envmap[i].getPixelsPtr()));
    }

    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
}

void EnvMapMeshRenderable::do_draw()
{
    //Location
    int denvmapLocation = m_shaderProgram->getUniformLocation("diffuseSampler");
    int senvmapLocation = m_shaderProgram->getUniformLocation("specularSampler");
    //Bind texture in Textured Unit 0
    if(denvmapLocation != ShaderProgram::null_location)
    {
        glcheck(glActiveTexture(GL_TEXTURE1)); // GL_TEXTURE0 is already occupied by texture
        glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, m_denvTexId));
        glcheck(glUniform1i(denvmapLocation, 1));
    }
    if(senvmapLocation != ShaderProgram::null_location)
    {
        glcheck(glActiveTexture(GL_TEXTURE2)); // GL_TEXTURE1 is already occupied by diffuse cubemap
        glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, m_senvTexId));
        glcheck(glUniform1i(senvmapLocation, 2));
    }
    
    TexturedLightedMeshRenderable::do_draw();

    // Release texture
    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
}
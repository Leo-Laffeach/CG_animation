#include "./../../include/texturing/CubeMapRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/Utils.hpp"
#include "./../../include/log.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <SFML/Graphics/Image.hpp>
#include <iostream>

CubeMapRenderable::~CubeMapRenderable()
{
    glcheck(glDeleteTextures(1, &m_texId));
}

CubeMapRenderable::CubeMapRenderable(
    ShaderProgramPtr program, 
    const std::string & dirname)
    : MeshRenderable(program, true)
{
    //Initialize geometry
    std::vector<glm::uvec3> uvec3_indices;
    getUnitIndexedCube(m_positions, m_normals, uvec3_indices);
    unpack(uvec3_indices, m_indices);
    m_colors.resize(m_positions.size(), glm::vec4(1.0,1.0,1.0,1.0));

    // Load images
    cmutils::load_cubemap(dirname, m_cubemap);

    // Low priority render this last !
    m_priority = -100;

    // Generate and send buffers
    glGenTextures(1, &m_texId);
    update_all_buffers();
}

void CubeMapRenderable::update_all_buffers(){
    MeshRenderable::update_all_buffers();
    update_textures_buffer();
}

void CubeMapRenderable::update_textures_buffer()
{
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_texId);
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)); 

    unsigned int width = m_cubemap[0].getSize().x;
    unsigned int height = m_cubemap[0].getSize().y;

    // Send data to gpu
    for (size_t i=0u;i<m_cubemap.size();++i){
        glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_cubemap[i].getPixelsPtr()));
    }
    // Equivalent to :
    // glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_cubemap[0].getPixelsPtr()));
    // glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_cubemap[1].getPixelsPtr()));
    // glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_cubemap[2].getPixelsPtr()));
    // glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_cubemap[3].getPixelsPtr()));
    // glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_cubemap[4].getPixelsPtr()));
    // glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_cubemap[5].getPixelsPtr()));
    
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
}

void CubeMapRenderable::do_draw()
{
    //Location
    int cubeMapLocation = m_shaderProgram->getUniformLocation("cubeMapSampler");
    //Bind texture in Textured Unit 0
    if(cubeMapLocation != ShaderProgram::null_location)
    {
        glcheck(glActiveTexture(GL_TEXTURE0));
        glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, m_texId));
    }
    
    glcheck(glDepthFunc(GL_LEQUAL));
    MeshRenderable::do_draw();
    glcheck(glDepthFunc(GL_LESS));

    // Release texture
    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
}
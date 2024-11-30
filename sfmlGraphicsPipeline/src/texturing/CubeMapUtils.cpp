#include "./../../include/texturing/CubeMapUtils.hpp"
#include <log.hpp>
#include <gl_helper.hpp>
#include <ShaderProgram.hpp>

cmutils::Cubemap cmutils::load_cubemap(const std::string & cubemap_dir)
{
    cmutils::Cubemap cubemap;
    for (std::size_t i=0u;i<cubemap.size();++i)
    {
        std::string filename = cubemap_dir + "/" + cmutils::face_names[i] + ".jpg";
        LOG(info, "[CubeMapUtils] Loading "<< filename);
        cubemap[i].loadFromFile(filename);
    }
    return cubemap;
}

void cmutils::load_cubemap(const std::string & cubemap_dir, cmutils::Cubemap & cubemap)
{
    for (std::size_t i=0u;i<cubemap.size();++i)
    {
        std::string filename = cubemap_dir + "/" + cmutils::face_names[i] + ".jpg";
        LOG(info, "[CubeMapUtils] Loading "<< filename);
        cubemap[i].loadFromFile(filename);
    }
}

void cmutils::save_cubemap(const cmutils::Cubemap & cubemap, const std::string & cubemap_dir)
{
    for (std::size_t i=0u;i<cubemap.size();++i)
    {
        std::string filename = cubemap_dir + "/" + cmutils::face_names[i] + ".jpg";
        LOG(info, "[CubeMapUtils] Writing "<< filename);
        cubemap[i].saveToFile(filename);
    }
}

unsigned int cmutils::send_quad()
{
    std::array<glm::vec2, 4> tcoords = {
        glm::vec2(-1,-1),
        glm::vec2(-1, 1),
        glm::vec2( 1, 1),
        glm::vec2( 1,-1),
    };
    unsigned int tBuffer;
    glcheck(glGenBuffers(1, &tBuffer)); //tcoords
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, tBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, tcoords.size()*sizeof(glm::vec2), tcoords.data(), GL_STATIC_DRAW));
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, 0));
    return tBuffer;
}

unsigned int cmutils::send_cubemap(const cmutils::Cubemap & cubemap)
{
    unsigned int cTid;
    glcheck(glGenTextures(1, &cTid));
    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, cTid));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)); 

    unsigned int width = cubemap[0].getSize().x;
    unsigned int height = cubemap[0].getSize().y;

    for (size_t i=0u;i<cubemap.size();++i){
        glcheck(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                             0,
                             GL_RGBA,
                             width,
                             height,
                             0,
                             GL_RGBA,
                             GL_UNSIGNED_BYTE,
                             (const GLvoid*)cubemap[i].getPixelsPtr()));
    }
    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
    return cTid;
}

void cmutils::cubemap_to_panorama_draw(const cmutils::Cubemap & cubemap, sf::RenderTexture & panorama_rt, unsigned int tBuffer, unsigned int cTid)
{
    ShaderProgram program = ShaderProgram(
        "../../sfmlGraphicsPipeline/shaders/cubeMapToPanoramaVertex.glsl",
        "../../sfmlGraphicsPipeline/shaders/cubeMapToPanoramaFragment.glsl");
    program.bind();
    int tcoordsLocation = program.getAttributeLocation("vTexCoord");
    int cubeMapLocation = program.getUniformLocation("cubeMapSampler");
    if(tcoordsLocation != ShaderProgram::null_location)
    {
        glcheck(glEnableVertexAttribArray(tcoordsLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, tBuffer));
        glcheck(glVertexAttribPointer(tcoordsLocation, 2, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }
    if(cubeMapLocation != ShaderProgram::null_location)
    {
        glcheck(glActiveTexture(GL_TEXTURE0));
        glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, cTid));
    }

    sf::Vector2u psize = panorama_rt.getSize();
    glcheck(glViewport(0, 0, psize.x, psize.y));
    panorama_rt.setActive(true);
    glcheck(glDrawArrays(GL_QUADS, 0, 4));
    panorama_rt.display();
    panorama_rt.setActive(false);

    glcheck(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
    glcheck(glDisableVertexAttribArray(tcoordsLocation));
    ShaderProgram::unbind();
}

void cmutils::blur_panorama_draw(sf::RenderTexture & panorama_rt, unsigned int tBuffer, unsigned int csize, unsigned int ksize)
{   
    ShaderProgram program = ShaderProgram(
        "../../sfmlGraphicsPipeline/shaders/blurVertex.glsl",
        "../../sfmlGraphicsPipeline/shaders/blurFragment.glsl");
    program.bind();
    
    panorama_rt.setSmooth(true);
    panorama_rt.setActive(true);

    int tcoordsLocation = program.getAttributeLocation("vTexCoord");
    int texsamplerLocation = program.getUniformLocation("texSampler");
    int ksizeLocation = program.getUniformLocation("ksize");
    int csizeLocation = program.getUniformLocation("csize");
    int directionLocation = program.getUniformLocation("direction");
    
    if(tcoordsLocation != ShaderProgram::null_location)
    {
        glcheck(glEnableVertexAttribArray(tcoordsLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, tBuffer));
        glcheck(glVertexAttribPointer(tcoordsLocation, 2, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    if (texsamplerLocation != ShaderProgram::null_location)
    {   
        glEnable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE0);
        sf::Texture::bind(&(panorama_rt.getTexture()));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));
        glUniform1i(texsamplerLocation, 0) ;
    }

    if(ksizeLocation != ShaderProgram::null_location)
        glcheck(glUniform1i(ksizeLocation, ksize));
    if(csizeLocation != ShaderProgram::null_location)
        glcheck(glUniform1i(csizeLocation, csize));
    if(directionLocation != ShaderProgram::null_location)
        glcheck(glUniform2f(directionLocation, 0, 1));

    sf::Vector2u psize = panorama_rt.getSize();
    glcheck(glViewport(0, 0, psize.x, psize.y));
    glcheck(glDrawArrays(GL_QUADS, 0, 4));
    panorama_rt.display();

    if (texsamplerLocation != ShaderProgram::null_location)
    {   
        glEnable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE0);
        sf::Texture::bind(&(panorama_rt.getTexture()));
        glUniform1i(texsamplerLocation, 0) ;
    }
    if(directionLocation != ShaderProgram::null_location)
        glcheck(glUniform2f(directionLocation, 1, 0));

    glcheck(glDrawArrays(GL_QUADS, 0, 4));
    panorama_rt.display();

    panorama_rt.setActive(false);
    sf::Texture::bind(0);
    glcheck(glDisableVertexAttribArray(tcoordsLocation));
    ShaderProgram::unbind();
}

void cmutils::panorama_to_cubemap_draw(const sf::RenderTexture & panorama_rt, cmutils::Cubemap & cubemap, unsigned int csize, unsigned int tBuffer)
{
    sf::RenderTexture cubemap_rt;
    cubemap_rt.create(csize, csize, sf::ContextSettings{ 0, 0, 4, 4, 0});
    
    ShaderProgram program = ShaderProgram(
        "../../sfmlGraphicsPipeline/shaders/panoramaToCubeMapVertex.glsl",
        "../../sfmlGraphicsPipeline/shaders/panoramaToCubeMapFragment.glsl");
    program.bind();

    int tcoordsLocation = program.getAttributeLocation("vTexCoord");
    int texsamplerLocation = program.getUniformLocation("panoramaSampler");
    
    if (texsamplerLocation != ShaderProgram::null_location)
    {   
        glEnable(GL_TEXTURE_2D);
        glActiveTexture(GL_TEXTURE0);
        sf::Texture::bind(&(panorama_rt.getTexture()));
        glUniform1i(texsamplerLocation, 0) ;
    }

    if(tcoordsLocation != ShaderProgram::null_location)
    {
        glcheck(glEnableVertexAttribArray(tcoordsLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, tBuffer));
        glcheck(glVertexAttribPointer(tcoordsLocation, 2, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    glcheck(glViewport(0, 0, csize, csize));
    cubemap_rt.setActive(true);

    for (int f=0;f<6;++f){
        int faceLocation = program.getUniformLocation("face");
        if(faceLocation != ShaderProgram::null_location)
            glcheck(glUniform1i(faceLocation, f));

        glcheck(glDrawArrays(GL_QUADS, 0, 4));
        cubemap_rt.display();
        cubemap[f] = cubemap_rt.getTexture().copyToImage();
    }
    
    cubemap_rt.setActive(false);
    sf::Texture::bind(0);
    glcheck(glDisableVertexAttribArray(tcoordsLocation));
    ShaderProgram::unbind();
}

cmutils::Cubemap cmutils::blur_cubemap(const cmutils::Cubemap & cubemap, unsigned int csize, unsigned int ksize)
{
    sf::RenderTexture panorama_rt;
    panorama_rt.create(4*csize, 3*csize, sf::ContextSettings{ 0, 0, sf::RenderTexture::getMaximumAntialiasingLevel(), 4, 0});
    unsigned int tBuffer = send_quad();
    unsigned int cTid = send_cubemap(cubemap);
    cmutils::cubemap_to_panorama_draw(cubemap, panorama_rt, tBuffer, cTid);
    // panorama_rt.getTexture().copyToImage().saveToFile("panorama_rt.jpg");
    cmutils::blur_panorama_draw(panorama_rt, tBuffer, csize, ksize);
    // panorama_rt.getTexture().copyToImage().saveToFile("panorama_rt_blurred.jpg");
    cmutils::Cubemap blurred_cubemap; 
    cmutils::panorama_to_cubemap_draw(panorama_rt, blurred_cubemap, csize, tBuffer);
    
    glcheck(glDeleteTextures(1, &cTid));
    glcheck(glDeleteBuffers(1, &tBuffer));
    return blurred_cubemap;
}

void cmutils::blur_cubemap_directory(const std::string & cubemap_dir, const std::string & blurred_cubemap_dir, unsigned int csize, unsigned int ksize)
{
    cmutils::Cubemap cubemap = cmutils::load_cubemap(cubemap_dir);
    cmutils::Cubemap blurred = cmutils::blur_cubemap(cubemap, csize, ksize);
    cmutils::save_cubemap(blurred, blurred_cubemap_dir);
}
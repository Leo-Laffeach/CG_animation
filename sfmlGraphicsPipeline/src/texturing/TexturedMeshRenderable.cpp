#include "./../../include/texturing/TexturedMeshRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Io.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

static const std::array<std::string, 5> wrap_option_names = {
    "GL_CLAMP_TO_EDGE with unchanged texture coordinates",
    "GL_REPEAT with a factor 10 on texture coordinates",
    "GL_MIRRORED_REPEAT with a factor 10 on texture coordinates",
    "GL_CLAMP_TO_EDGE with a factor 10 on texture coordinates ",
    "GL_CLAMP_TO_BORDER with a factor 10 on texture coordinates and a default color"
};

static const std::array<std::string, 3> filter_option_names = {
    "GL_NEAREST",
    "GL_LINEAR",
    "GL_LINEAR_MIPMAP_LINEAR"
};

TexturedMeshRenderable::~TexturedMeshRenderable()
{
    glcheck(glDeleteBuffers(1, &m_tBuffer));
    glcheck(glDeleteTextures(1, &m_texId));
}

TexturedMeshRenderable::TexturedMeshRenderable(
    ShaderProgramPtr program,
    const std::string & mesh_filename,
    const std::string & texture_filename) :
    MeshRenderable(program, mesh_filename), // Should initialize m_tcoords trought read_obj...
    m_tBuffer(0), m_texId(0), m_wrap_option(0), m_filter_option(0)
{
    m_image.loadFromFile(texture_filename);
    if (m_tcoords.size() != m_positions.size()){
        m_tcoords.resize(m_positions.size(), glm::vec2(0.0));
    }
    m_original_tcoords = m_tcoords; // m_tcoords is already loaded from MeshRenderable ctor
    m_image.flipVertically();
    gen_buffers();
    update_buffers();
}

TexturedMeshRenderable::TexturedMeshRenderable(
    ShaderProgramPtr program,
    const std::vector< glm::vec3 > & positions,
    const std::vector< unsigned int > & indices,
    const std::vector< glm::vec3 > & normals,
    const std::vector< glm::vec4 > & colors,
    const sf::Image & image,
    const std::vector< glm::vec2 > & tcoords) :
    MeshRenderable(program, positions, indices, normals, colors),
    m_tBuffer(0), m_texId(0), m_image(image), m_wrap_option(0), m_filter_option(0)
{
    m_tcoords = tcoords;
    m_original_tcoords = tcoords;
    gen_buffers();
    update_buffers();
}

TexturedMeshRenderable::TexturedMeshRenderable(
    ShaderProgramPtr program,
    const std::vector< glm::vec3 > & positions,
    const std::vector< glm::vec3 > & normals,
    const std::vector< glm::vec4 > & colors,
    const sf::Image & image,
    const std::vector< glm::vec2 > & tcoords) :
    MeshRenderable(program, positions, normals, colors),
    m_tBuffer(0), m_texId(0), m_image(image), m_wrap_option(0), m_filter_option(0)
{
    m_tcoords = tcoords;
    m_original_tcoords = tcoords;
    gen_buffers();
    update_buffers();
}

TexturedMeshRenderable::TexturedMeshRenderable(ShaderProgramPtr prog, bool indexed) :
    MeshRenderable(prog, indexed),
    m_tBuffer(0), m_texId(0), m_wrap_option(0), m_filter_option(0)
{
    gen_buffers();
}

void TexturedMeshRenderable::gen_buffers()
{
    glcheck(glGenBuffers(1, &m_tBuffer)); //texture coordinates
    glcheck(glGenTextures(1, &m_texId));
}

void TexturedMeshRenderable::update_buffers()
{
    update_tcoords_buffer();
    update_texture_buffer();
}

void TexturedMeshRenderable::update_all_buffers(){
    MeshRenderable::update_all_buffers();
    update_buffers();
}

void TexturedMeshRenderable::update_texture_buffer(){
    glcheck(glBindTexture(GL_TEXTURE_2D, m_texId));
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    // send the texture
    glcheck(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_image.getSize().x, m_image.getSize().y, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)m_image.getPixelsPtr()));

    //Release the texture
    glcheck(glBindTexture(GL_TEXTURE_2D, 0));
}

void TexturedMeshRenderable::update_tcoords_buffer(){
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_tBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_tcoords.size()*sizeof(glm::vec2), m_tcoords.data(), GL_STATIC_DRAW));
}

void TexturedMeshRenderable::do_draw()
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

    // Release texture
    glcheck(glBindTexture(GL_TEXTURE_2D, 0));
    // Release tcoord vertex attribute
    if(texcoordLocation != ShaderProgram::null_location)
        glcheck(glDisableVertexAttribArray(texcoordLocation))
}

std::vector< glm::vec2 > & TexturedMeshRenderable::tcoords()
{
    return m_tcoords;
}

const std::vector< glm::vec2 > & TexturedMeshRenderable::tcoords() const
{
    return m_tcoords;
}

sf::Image & TexturedMeshRenderable::image()
{
    return m_image;
}

const sf::Image & TexturedMeshRenderable::image() const
{
    return m_image;
}

void TexturedMeshRenderable::updateTextureOption()
{
    //Resize texture coordinates factor
    float factor=10.0;

    //Bind the texture
    glBindTexture(GL_TEXTURE_2D, m_texId);

    //Textured options
    if(m_wrap_option==0)
    {
        m_tcoords = m_original_tcoords;
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    }
    else if(m_wrap_option==1)
    {
        for(size_t i=0; i<m_tcoords.size(); ++i)
            m_tcoords[i] = factor*m_original_tcoords[i];
        
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));
    }
    else if(m_wrap_option==2)
    {
        for(size_t i=0; i<m_tcoords.size(); ++i)
            m_tcoords[i] = factor*m_original_tcoords[i];

        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));
    }
    else if(m_wrap_option==3){
        for(size_t i=0; i<m_tcoords.size(); ++i)
            m_tcoords[i] = factor*m_original_tcoords[i] - glm::vec2(factor/2.0, factor/2.0);

        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    }
    else if(m_wrap_option==4){
        for(size_t i=0; i<m_tcoords.size(); ++i)
            m_tcoords[i] = factor*m_original_tcoords[i] - glm::vec2(factor/2.0, factor/2.0);

        float borderColor[] = { 0.7f, 0.6f, 0.8f, 1.0f };
        glcheck(glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER));
    }

    if(m_filter_option==0)
    {
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
    }
    else if(m_filter_option==1)
    {
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    }
    else if(m_filter_option==2)
    {
        glcheck(glGenerateMipmap(GL_TEXTURE_2D));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
        glcheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    }

    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_tBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_tcoords.size()*sizeof(glm::vec2), m_tcoords.data(), GL_STATIC_DRAW));

    //Release the texture
    glBindTexture(GL_TEXTURE_2D, 0);
}

void TexturedMeshRenderable::do_keyPressedEvent( sf::Event& e )
{
    if(e.key.code == sf::Keyboard::F6){
        m_wrap_option = ++m_wrap_option % 5;
        LOG(info, "Texture wrapping set to : "<<wrap_option_names[m_wrap_option]);
    }

    if(e.key.code == sf::Keyboard::F7){
        m_filter_option = ++m_filter_option % 3;
        LOG(info, "Texture filtering set to : "<<filter_option_names[m_filter_option]);
    }

    updateTextureOption();
}

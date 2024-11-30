#ifndef MIPMAP_CUBE_RENDERABLE_HPP
#define MIPMAP_CUBE_RENDERABLE_HPP

#include "./../MeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

class MipMapCubeRenderable : public MeshRenderable
{
public :
    ~MipMapCubeRenderable();
    MipMapCubeRenderable(ShaderProgramPtr shaderProgram, const std::vector<std::string>& filenames);
    void update_texture_buffer();
    void update_tcoords_buffer();
    void update_all_buffers();

protected:
    void do_draw();

private:
    void do_keyPressedEvent( sf::Event& e );
    void updateTextureOption();
    void gen_buffers();
    void update_buffers();

    std::vector< sf::Image > m_images; 

    // std::vector< glm::vec2 > m_tcoords; Already has from MeshRenderable

    unsigned int m_tBuffer;
    unsigned int m_texId;

    unsigned int m_mipmapOption;
    
};

typedef std::shared_ptr<MipMapCubeRenderable> MipMapCubeRenderablePtr;

#endif

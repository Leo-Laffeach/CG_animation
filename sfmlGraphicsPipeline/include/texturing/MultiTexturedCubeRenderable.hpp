#ifndef MULTI_TEXTURED_CUBE_RENDERABLE_HPP
#define MULTI_TEXTURED_CUBE_RENDERABLE_HPP

#include "./../MeshRenderable.hpp"
#include "./../lighting/Material.hpp"
#include <vector>
#include <glm/glm.hpp>

class MultiTexturedCubeRenderable : public MeshRenderable
{
public :
    ~MultiTexturedCubeRenderable();
    MultiTexturedCubeRenderable(ShaderProgramPtr shaderProgram, const std::string &filename1, const std::string& filename2);
    void update_textures_buffer();
    void update_tcoords_buffer();
    void update_all_buffers();

protected:
    void do_draw();

private:
    void gen_buffers();
    void update_buffers();

    unsigned int m_tBuffer;
    unsigned int m_texId1, m_texId2;
    sf::Image m_image1, m_image2;
};

typedef std::shared_ptr<MultiTexturedCubeRenderable> MultiTexturedCubeRenderablePtr;

#endif

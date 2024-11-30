#ifndef CUBEMAP_RENDERABLE_HPP
#define CUBEMAP_RENDERABLE_HPP

#include "MeshRenderable.hpp"
#include <vector>
#include <array>
#include <glm/glm.hpp>
#include <texturing/CubeMapUtils.hpp>

class CubeMapRenderable : public MeshRenderable
{
public:
    ~CubeMapRenderable();
    CubeMapRenderable( ShaderProgramPtr program, const std::string & dirname);
    void update_all_buffers();
    void update_textures_buffer();

private:
    void do_draw();

    cmutils::Cubemap m_cubemap;
    unsigned int m_texId;
};

typedef std::shared_ptr<CubeMapRenderable> CubeMapRenderablePtr;

#endif

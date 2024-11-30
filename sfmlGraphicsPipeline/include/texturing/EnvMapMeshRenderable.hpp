#ifndef ENVMAPMESH_RENDERABLE_HPP
#define ENVMAPMESH_RENDERABLE_HPP

#include "TexturedLightedMeshRenderable.hpp"
#include <vector>
#include <array>
#include <glm/glm.hpp>
#include <texturing/CubeMapUtils.hpp>

class EnvMapMeshRenderable : public TexturedLightedMeshRenderable
{
public:
    ~EnvMapMeshRenderable();
    EnvMapMeshRenderable(
    ShaderProgramPtr program,
        const std::string & mesh_filename,
        const MaterialPtr & mat,
        const std::string & texture_filename,
        const std::string & diffuse_envmap_dir,
        const std::string & specular_envmap_dir);
    void update_all_buffers();
    void update_textures_buffer();

private:
    void do_draw();

    cmutils::Cubemap m_diffuse_envmap;
    cmutils::Cubemap m_specular_envmap;
    unsigned int m_denvTexId;
    unsigned int m_senvTexId;
};

typedef std::shared_ptr<EnvMapMeshRenderable> EnvMapMeshRenderablePtr;

#endif

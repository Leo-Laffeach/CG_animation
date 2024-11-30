#ifndef DIRECTIONAL_LIGHT_RENDERABLE_HPP
#define DIRECTIONAL_LIGHT_RENDERABLE_HPP

#include "./../MeshRenderable.hpp"
#include "Light.hpp"

#include <vector>
#include <glm/glm.hpp>

class DirectionalLightRenderable : public MeshRenderable
{
public:
    ~DirectionalLightRenderable();
    DirectionalLightRenderable( ShaderProgramPtr program, DirectionalLightPtr light);

private:
    void do_animate(float time);

    DirectionalLightPtr m_light;
};

typedef std::shared_ptr<DirectionalLightRenderable> DirectionalLightRenderablePtr;

#endif

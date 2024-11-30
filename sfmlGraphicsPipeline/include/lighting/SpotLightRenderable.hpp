#ifndef SPOT_LIGHT_RENDERABLE_HPP
#define SPOT_LIGHT_RENDERABLE_HPP

#include "./../MeshRenderable.hpp"
#include "Light.hpp"

#include <vector>
#include <glm/glm.hpp>

class SpotLightRenderable : public MeshRenderable
{
public:
    ~SpotLightRenderable();
    SpotLightRenderable( const ShaderProgramPtr & prog, const SpotLightPtr & light, unsigned int slices=20u);

private:
    void do_animate(float time);

    SpotLightPtr m_light;
};

typedef std::shared_ptr<SpotLightRenderable> SpotLightRenderablePtr;

#endif

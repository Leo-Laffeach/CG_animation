#ifndef POINT_LIGHT_RENDERABLE_HPP
#define POINT_LIGHT_RENDERABLE_HPP

#include "./../MeshRenderable.hpp"
#include "Light.hpp"

#include <vector>
#include <glm/glm.hpp>

class PointLightRenderable : public MeshRenderable
{
public:
    ~PointLightRenderable();
    PointLightRenderable( const ShaderProgramPtr & program, const PointLightPtr & light, unsigned int strips=10u, unsigned int slices=20u);

private:
    void do_animate(float time);

    PointLightPtr m_light;
};

typedef std::shared_ptr<PointLightRenderable> PointLightRenderablePtr;

#endif

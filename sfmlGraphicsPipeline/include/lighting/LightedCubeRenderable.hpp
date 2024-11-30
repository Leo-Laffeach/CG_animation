#ifndef LIGHTED_CUBE_RENDERABLE_HPP
#define LIGHTED_CUBE_RENDERABLE_HPP

#include "./../lighting/LightedMeshRenderable.hpp"
#include "./../lighting/Material.hpp"
#include <vector>
#include <glm/glm.hpp>

class LightedCubeRenderable : public LightedMeshRenderable
{
public:
    ~LightedCubeRenderable();
    LightedCubeRenderable( ShaderProgramPtr program, bool indexed, const MaterialPtr& material);
};

typedef std::shared_ptr<LightedCubeRenderable> LightedCubeRenderablePtr;

#endif

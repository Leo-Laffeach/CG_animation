#ifndef LIGHTED_CYLINDER_RENDERABLE_HPP
#define LIGHTED_CYLINDER_RENDERABLE_HPP

#include "./../lighting/LightedMeshRenderable.hpp"
#include "./../lighting/Material.hpp"
#include <vector>
#include <glm/glm.hpp>

class LightedCylinderRenderable : public LightedMeshRenderable
{
public:
    ~LightedCylinderRenderable();
    LightedCylinderRenderable(ShaderProgramPtr prog, bool indexed, const MaterialPtr & mat, unsigned int slice=30u, bool vertex_normals=true);
};

typedef std::shared_ptr<LightedCylinderRenderable> LightedCylinderRenderablePtr;

#endif

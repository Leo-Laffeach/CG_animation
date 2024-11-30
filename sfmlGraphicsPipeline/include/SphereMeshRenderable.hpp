#ifndef SPHERE_MESH_RENDERABLE_HPP
#define SPHERE_MESH_RENDERABLE_HPP

#include "MeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

class SphereMeshRenderable : public MeshRenderable
{
public:
    ~SphereMeshRenderable();
    SphereMeshRenderable( ShaderProgramPtr program, bool indexed, unsigned int strips=10u, unsigned int slices=20u, bool vertex_normals=true);
};

typedef std::shared_ptr<SphereMeshRenderable> SphereMeshRenderablePtr;

#endif

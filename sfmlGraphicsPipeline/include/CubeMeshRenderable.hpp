#ifndef CUBE_MESH_RENDERABLE_HPP
#define CUBE_MESH_RENDERABLE_HPP

#include "MeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

class CubeMeshRenderable : public MeshRenderable
{
    public:
        ~CubeMeshRenderable();
        CubeMeshRenderable( ShaderProgramPtr program, bool indexed);
};

typedef std::shared_ptr<CubeMeshRenderable> CubeMeshRenderablePtr;

#endif

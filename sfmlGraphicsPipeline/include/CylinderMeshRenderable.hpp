#ifndef CYLINDER_MESH_RENDERABLE_HPP
#define CYLINDER_MESH_RENDERABLE_HPP

#include "MeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

class CylinderMeshRenderable : public MeshRenderable
{
public:
    ~CylinderMeshRenderable();
    CylinderMeshRenderable( ShaderProgramPtr program, bool indexed, unsigned int slice=20u, bool vertex_normals=true);
};

typedef std::shared_ptr<CylinderMeshRenderable> CylinderMeshRenderablePtr;

#endif

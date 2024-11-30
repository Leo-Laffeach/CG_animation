#ifndef QUAD_MESH_RENDERABLE_HPP
#define QUAD_MESH_RENDERABLE_HPP

#include "MeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

class QuadMeshRenderable : public MeshRenderable
{
    public:
        ~QuadMeshRenderable();
        QuadMeshRenderable( 
            ShaderProgramPtr program,
            const glm::vec3 & p1 = glm::vec3(-1,-1,0),
            const glm::vec3 & p2 = glm::vec3(1, -1,0),
            const glm::vec3 & p3 = glm::vec3(1, 1, 0),
            const glm::vec3 & p4 = glm::vec3(-1,1, 0),
            const glm::vec4 & color = glm::vec4(0.3,0.4,0.8,1));
};

typedef std::shared_ptr<QuadMeshRenderable> QuadMeshRenderablePtr;

#endif

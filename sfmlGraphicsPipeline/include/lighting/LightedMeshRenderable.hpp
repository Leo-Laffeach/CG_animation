#ifndef LIGHTED_MESH_RENDERABLE_HPP
#define LIGHTED_MESH_RENDERABLE_HPP

#include "./../MeshRenderable.hpp"
#include "./../lighting/Material.hpp"
#include "./../lighting/Light.hpp"

#include <string>
#include <vector>
#include <glm/glm.hpp>

class LightedMeshRenderable : public MeshRenderable
{
    public:
        ~LightedMeshRenderable();
        
        LightedMeshRenderable(ShaderProgramPtr program,
                              const std::string & filename,
                              const MaterialPtr & material);
        
        LightedMeshRenderable(ShaderProgramPtr shaderProgram,
                              const std::vector< glm::vec3 > & positions,
                              const std::vector< unsigned int > & indices,
                              const std::vector< glm::vec3 > & normals,
                              const std::vector< glm::vec4 > & colors,
                              const MaterialPtr & material);

        LightedMeshRenderable(ShaderProgramPtr shaderProgram,
                              const std::vector< glm::vec3 > & positions,
                              const std::vector< glm::vec3 > & normals,
                              const std::vector< glm::vec4 > & colors,
                              const MaterialPtr & material);
    
        const MaterialPtr & getMaterial() const;
        void setMaterial(const MaterialPtr &);

    protected:
        LightedMeshRenderable(ShaderProgramPtr shaderProgram, bool indexed, const MaterialPtr & material);

        void do_draw();

    private:
        MaterialPtr m_material;
};

typedef std::shared_ptr<LightedMeshRenderable> LightedMeshRenderablePtr;

#endif

#ifndef TEXTURED_LIGHTED_MESH_RENDERABLE_HPP
#define TEXTURED_LIGHTED_MESH_RENDERABLE_HPP

#include "./../texturing/TexturedMeshRenderable.hpp"
#include "./../lighting/Material.hpp"

#include <string>
#include <vector>
#include <glm/glm.hpp>

// This class could be made using multiple inheritance 
// (derived from LightedMeshRenderable AND TextureMeshRenderable).
// We will not do that since MeshRenderable::do_draw() would be called two times.
// Inheritance is not the prefered method for designing a 3D engine anyway.
// See Entity Component System (more difficult to setup, but more convenient)

class TexturedLightedMeshRenderable : public TexturedMeshRenderable
{
    public:
        ~TexturedLightedMeshRenderable();
            
        TexturedLightedMeshRenderable(ShaderProgramPtr program,
                                      const std::string & mesh_filename,
                                      const MaterialPtr & material,
                                      const std::string & texture_filename);
        
        TexturedLightedMeshRenderable(ShaderProgramPtr shaderProgram,
                                      const std::vector< glm::vec3 > & positions,
                                      const std::vector< unsigned int > & indices,
                                      const std::vector< glm::vec3 > & normals,
                                      const std::vector< glm::vec4 > & colors,
                                      const MaterialPtr & material,
                                      const sf::Image & image,
                                      const std::vector< glm::vec2 > & tcoords);

        TexturedLightedMeshRenderable(ShaderProgramPtr shaderProgram,
                                      const std::vector< glm::vec3 > & positions,
                                      const std::vector< glm::vec3 > & normals,
                                      const std::vector< glm::vec4 > & colors,
                                      const MaterialPtr & material,
                                      const sf::Image & image,
                                      const std::vector< glm::vec2 > & tcoords);

    
        const MaterialPtr & getMaterial() const;
        void setMaterial(const MaterialPtr &);

    protected:

        void do_draw();

    private:
        MaterialPtr m_material;
};

typedef std::shared_ptr<TexturedLightedMeshRenderable> TexturedLightedMeshRenderablePtr;

#endif

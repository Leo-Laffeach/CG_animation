#ifndef TEXTURED_MESH_RENDERABLE_HPP
#define TEXTURED_MESH_RENDERABLE_HPP

#include "./../MeshRenderable.hpp"

#include <string>
#include <vector>
#include <glm/glm.hpp>

class TexturedMeshRenderable : public MeshRenderable
{
    public:
        ~TexturedMeshRenderable();
            
        TexturedMeshRenderable(ShaderProgramPtr program,
                                      const std::string & mesh_filename,
                                      const std::string & texture_filename);
        
        TexturedMeshRenderable(ShaderProgramPtr shaderProgram,
                                      const std::vector< glm::vec3 > & positions,
                                      const std::vector< unsigned int > & indices,
                                      const std::vector< glm::vec3 > & normals,
                                      const std::vector< glm::vec4 > & colors,
                                      const sf::Image & image,
                                      const std::vector< glm::vec2 > & tcoords);

        TexturedMeshRenderable(ShaderProgramPtr shaderProgram,
                                      const std::vector< glm::vec3 > & positions,
                                      const std::vector< glm::vec3 > & normals,
                                      const std::vector< glm::vec4 > & colors,
                                      const sf::Image & image,
                                      const std::vector< glm::vec2 > & tcoords);
    
    std::vector< glm::vec2 > & tcoords();
    const std::vector< glm::vec2 > & tcoords() const;
    sf::Image & image();
    const sf::Image & image() const;
    void update_texture_buffer();
    void update_tcoords_buffer();
    void update_all_buffers();
    
    protected:
        TexturedMeshRenderable(ShaderProgramPtr shaderProgram, bool indexed);
        void do_draw();

        unsigned int m_tBuffer;
        unsigned int m_texId;
        sf::Image m_image;
        // std::vector< glm::vec2 > m_tcoords; Already in MeshRenderable
        std::vector< glm::vec2 > m_original_tcoords;

    private:
        void do_keyPressedEvent( sf::Event& e );
        void updateTextureOption();
        void gen_buffers();
        void update_buffers();

        unsigned int m_wrap_option;
        unsigned int m_filter_option;
};

typedef std::shared_ptr<TexturedMeshRenderable> TexturedMeshRenderablePtr;

#endif

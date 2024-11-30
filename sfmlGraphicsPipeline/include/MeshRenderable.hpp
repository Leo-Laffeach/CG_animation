#ifndef MESH_RENDERABLE_HPP
#define MESH_RENDERABLE_HPP

#include "KeyframedHierarchicalRenderable.hpp"

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <GL/glew.h>

class MeshRenderable : public KeyframedHierarchicalRenderable
{
    public:
        virtual ~MeshRenderable();

        MeshRenderable(ShaderProgramPtr program,
                       const std::string & mesh_filename);

        MeshRenderable(ShaderProgramPtr program,
                       const std::vector< glm::vec3 > & positions,
                       const std::vector< unsigned int > & indices, 
                       const std::vector< glm::vec3 > & normals,
                       const std::vector< glm::vec4 > & colors);
        
        MeshRenderable(ShaderProgramPtr program,
                       const std::vector< glm::vec3 > & positions,
                       const std::vector< glm::vec3 > & normals,
                       const std::vector< glm::vec4 > & colors);

        void update_positions_buffer();
        void update_colors_buffer();
        void update_normals_buffer();
        void update_indices_buffer();
        virtual void update_all_buffers();

    protected:
        void do_draw();
        MeshRenderable(ShaderProgramPtr program, bool indexed);

        GLenum m_mode;
        std::vector< glm::vec3 > m_positions;
        std::vector< glm::vec3 > m_normals;
        std::vector< glm::vec4 > m_colors;
        std::vector< unsigned int > m_indices;
        bool m_indexed;
        std::vector< glm::vec2 > m_tcoords;
        
        unsigned int m_pBuffer;
        unsigned int m_cBuffer;
        unsigned int m_nBuffer;
        unsigned int m_iBuffer;

    private:
        void gen_buffers();
        void update_buffers();
        void set_random_colors();


};

typedef std::shared_ptr<MeshRenderable> MeshRenderablePtr;

#endif

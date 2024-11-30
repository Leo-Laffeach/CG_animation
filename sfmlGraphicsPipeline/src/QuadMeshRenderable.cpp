#include "./../include/QuadMeshRenderable.hpp"
#include "./../include/gl_helper.hpp"
#include "./../include/log.hpp"
#include "./../include/Utils.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

QuadMeshRenderable:: QuadMeshRenderable( 
    ShaderProgramPtr program,
    const glm::vec3 & p1,
    const glm::vec3 & p2,
    const glm::vec3 & p3,
    const glm::vec3 & p4,
    const glm::vec4 & color) : 
    MeshRenderable(program, false)
{
    m_mode = GL_QUADS;
    m_positions = {p1, p2, p3, p4};
    m_colors.resize(4, color);
    m_normals.resize(4, glm::vec3(0,0,1));
    update_all_buffers();
}

QuadMeshRenderable::~QuadMeshRenderable()
{}

# include "../../include/lighting/LightedCylinderRenderable.hpp"
# include "../../include/gl_helper.hpp"
# include "../../include/Utils.hpp"

# include <glm/gtc/type_ptr.hpp>
# include <GL/glew.h>

LightedCylinderRenderable::~LightedCylinderRenderable()
{}

LightedCylinderRenderable::LightedCylinderRenderable( ShaderProgramPtr prog, bool indexed, const MaterialPtr & mat, unsigned int slices, bool vertex_normals)
    : LightedMeshRenderable( prog, indexed, mat )
{
    if (indexed){
        std::vector<glm::uvec3> indices;
        getUnitIndexedCylinder(m_positions, m_normals, indices, slices);
        // getUniIndexedCube fills a std::vector<glm::uvec3>,
        // but m_indices is a std::vector<unsigned int>.
        // We need to unpack the values.
        unpack(indices, m_indices);
    }else{
        getUnitCylinder(m_positions, m_normals, m_tcoords, slices, vertex_normals);
    }
    m_colors.resize(m_positions.size(), glm::vec4(0));
    for (size_t i=0; i<m_colors.size(); ++i)
        m_colors[i] = randomColor();

    update_all_buffers();
}
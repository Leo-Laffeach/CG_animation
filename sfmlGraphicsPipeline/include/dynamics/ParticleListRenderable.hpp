# ifndef PARTICLELISTRENDERABLE_HPP_
# define PARTICLELISTRENDERABLE_HPP_

#include "Particle.hpp"
#include "../HierarchicalRenderable.hpp"
#include "../Utils.hpp"
#include "../gl_helper.hpp"
#include "../log.hpp"

#include <vector>
#include <glm/glm.hpp>
#include <GL/glew.h>

/**@brief Draw a list of particles.
 *
 * This class is used to draw a list of particle. This is more efficient
 * than to render individually each particle. We could do even more
 * efficient but it is beyond the objectives of those practicals.
 */
class ParticleListRenderable :
        public HierarchicalRenderable
{
public:
    ~ParticleListRenderable();
    /**@brief Build a renderable to render a set of particles.
     *
     * Build a renderable to render a set of particles.
     * @param program The shader program used to render the particles.
     * @param particles The set of particles to render.
     */
    ParticleListRenderable(ShaderProgramPtr program, std::vector<ParticlePtr>& particles, unsigned int strips=10u, unsigned int slices=20u);
    
    void update_all_buffers();


protected:
    void do_draw();

private:

    void genbuffers();
    void update_positions_buffer();
    void update_colors_buffer();
    void update_normals_buffer();
    void update_indices_buffer();
    void update_instances_data_buffer();

    std::vector< glm::vec3 > m_positions;
    std::vector< glm::vec3 > m_normals;
    std::vector< glm::vec4 > m_colors;
    std::vector< unsigned int > m_indices;
        
    unsigned int m_pBuffer;
    unsigned int m_cBuffer;
    unsigned int m_nBuffer;
    unsigned int m_iBuffer;
    unsigned int m_idBuffer; // instance data

    std::vector< ParticlePtr > m_particles;
};

typedef std::shared_ptr<ParticleListRenderable> ParticleListRenderablePtr;

# endif 

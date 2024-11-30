#ifndef PARTICLE_RENDERABLE_HPP
#define PARTICLE_RENDERABLE_HPP

#include "../MeshRenderable.hpp"
#include "Particle.hpp"

#include <vector>
#include <glm/glm.hpp>

/**@brief Render a particle to the screen.
 *
 * Render a particle to the screen. Since a particle is modeled by
 * a ball, this renderable simply render the corresponding ball. If
 * you have more than one renderable, have a look to ParticleListRenderable.
 */
class ParticleRenderable : public MeshRenderable
{
public:
    ~ParticleRenderable();
    /**@brief Build a particle renderable.
        *
        * Build a renderable to render a particle.
        * @param program The shader program used to render the particle.
        * @param particle The particle to render.
        */
    ParticleRenderable( ShaderProgramPtr program, const ParticlePtr & particle, unsigned int strips=10u, unsigned int slices=20u);

protected:
    void do_draw();

private:
    ParticlePtr m_particle;

};

typedef std::shared_ptr<ParticleRenderable> ParticleRenderablePtr;

#endif //PARTICLE_RENDERABLE_HPP

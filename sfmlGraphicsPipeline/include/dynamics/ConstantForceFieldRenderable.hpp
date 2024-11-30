#ifndef CONSTANT_RENDERABLE_HPP
#define CONSTANT_RENDERABLE_HPP

#include "../MeshRenderable.hpp"
#include "ConstantForceField.hpp"

/**@brief Render a constant force field.
 *
 * This class is used to render a constant force field.
 */
class ConstantForceFieldRenderable : public MeshRenderable
{
public:
    ~ConstantForceFieldRenderable();
    /**@brief Build a renderable for a constant force field.
     *
     * Create a renderable for a constant force field.
     * @param program Shader program to use to render the constant force field.
     * @param forceField The constant force field to render.
     */
    ConstantForceFieldRenderable( ShaderProgramPtr program, ConstantForceFieldPtr forceField);
protected:
    void do_draw();
    ConstantForceFieldPtr m_forceField;

private:
    void update_particle_positions();
};

typedef std::shared_ptr<ConstantForceFieldRenderable> ConstantForceFieldRenderablePtr;

#endif // CONSTANT_RENDERABLE_HPP


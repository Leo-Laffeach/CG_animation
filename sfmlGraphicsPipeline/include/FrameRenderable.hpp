// #endif
#ifndef FRAME_RENDERABLE_HPP
#define FRAME_RENDERABLE_HPP
/**@file
 * @brief Define a class to render the world frame.
 *
 * This file defines the class FrameRenderable to render the world frame.
 */
#include "MeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

/**@brief Render the world frame.
 *
 * This class is the first renderable you will use (see the Tutorial 03 of
 * practical 01). It will render the world frame on the screen: a red line
 * for the X axis, a green line for the Y axis and a blue line for the Z
 * axis. Have a look at the source: we documented it to help you to understand
 * the details.
 */

// A renderable must implement the Renderable interface, as such we make this
// class inherit publicly from Renderable.
class FrameRenderable : public MeshRenderable
{
// Only the constructor and the destructor are meant to be called outside of
// the class. This is why they constitute the only public methods.
public :

    virtual ~FrameRenderable();

    FrameRenderable(ShaderProgramPtr shaderProgram);

private:
    void do_draw();
};

typedef std::shared_ptr<FrameRenderable> FrameRenderablePtr;

#endif

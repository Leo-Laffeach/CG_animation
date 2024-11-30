#ifndef TEXTURED_TRIANGLE_RENDERABLE_HPP
#define TEXTURED_TRIANGLE_RENDERABLE_HPP

#include "./../texturing/TexturedMeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

class TexturedTriangleRenderable : public TexturedMeshRenderable
{
public :
    ~TexturedTriangleRenderable();
    TexturedTriangleRenderable(ShaderProgramPtr shaderProgram, const std::string & filename);
};

typedef std::shared_ptr<TexturedTriangleRenderable> TexturedTriangleRenderablePtr;

#endif

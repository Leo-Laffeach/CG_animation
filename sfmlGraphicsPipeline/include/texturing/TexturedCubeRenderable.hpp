#ifndef TEXTURED_CUBE_RENDERABLE_HPP
#define TEXTURED_CUBE_RENDERABLE_HPP

#include "./../texturing/TexturedMeshRenderable.hpp"
#include <vector>
#include <glm/glm.hpp>

class TexturedCubeRenderable : public TexturedMeshRenderable
{
public :
    ~TexturedCubeRenderable();
    TexturedCubeRenderable(ShaderProgramPtr shaderProgram, const std::string& filename);
};

typedef std::shared_ptr<TexturedCubeRenderable> TexturedCubeRenderablePtr;

#endif

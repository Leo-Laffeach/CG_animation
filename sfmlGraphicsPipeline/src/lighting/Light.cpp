#include "./../../include/lighting/Light.hpp"
#include "./../../include/log.hpp"
#include <glm/gtc/type_ptr.hpp>

// Let's give the lights the same "forward direction" as our camera
glm::vec3 Light::base_forward = glm::vec3(0, 0, -1);

bool Light::sendToGPU(const ShaderProgramPtr& program, const LightPtr & light)
{
    if (program==nullptr || light==nullptr)
        return false;
    return light->sendToGPU(program, light->lightName());
}

bool Light::sendToGPU(const ShaderProgramPtr& program, const std::string & identifier) const
{
    bool success = true;
    int location = ShaderProgram::null_location;

    location = program->getUniformLocation(identifier+".ambient");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform3fv(location, 1, glm::value_ptr(m_ambient)));
    }else { success = false; }

    location = program->getUniformLocation(identifier+".diffuse");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform3fv(location, 1, glm::value_ptr(m_diffuse)));
    }else { success = false; }

    location = program->getUniformLocation(identifier+".specular");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform3fv(location, 1, glm::value_ptr(m_specular)));
    }else { success = false; }

    return success;
}

bool DirectionalLight::sendToGPU(const ShaderProgramPtr& program, const std::string & identifier) const
{
    bool success = Light::sendToGPU(program, identifier);
    int location = ShaderProgram::null_location;
    
    location = program->getUniformLocation(identifier+".direction");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform3fv(location, 1, glm::value_ptr(m_direction)));
    }else { success = false; }

    return success;
}

bool PointLight::sendToGPU(const ShaderProgramPtr& program, const std::string & identifier) const
{
    
    bool success = Light::sendToGPU(program, identifier);
    int location = ShaderProgram::null_location;

    location = program->getUniformLocation(identifier+".position");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform3fv(location, 1, glm::value_ptr(m_position)));
    }else { success = false; }
        
    location = program->getUniformLocation(identifier+".constant");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform1f(location, m_constant));
    }else { success = false; }
        
    location = program->getUniformLocation(identifier+".linear");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform1f(location, m_linear));
    }else { success = false; }

    location = program->getUniformLocation(identifier+".quadratic");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform1f(location, m_quadratic));
    }else { success = false; }

    return success;
}

bool SpotLight::sendToGPU(const ShaderProgramPtr& program, const std::string & identifier) const
{
    bool success = PointLight::sendToGPU(program, identifier);
    int location = ShaderProgram::null_location;

    location = program->getUniformLocation(identifier+".spotDirection");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform3fv(location, 1, glm::value_ptr(m_spotDirection)));
    }else { success = false; }

    location = program->getUniformLocation(identifier+".innerCutOff");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform1f(location, m_innerCutOff));
    }else { success = false; }

    location = program->getUniformLocation(identifier+".outerCutOff");
    if(location!=ShaderProgram::null_location){
        glcheck(glUniform1f(location, m_outerCutOff));
    }else { success = false; }

    return success;
}
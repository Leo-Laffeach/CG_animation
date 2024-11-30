#ifndef LIGHT_HPP
#define LIGHT_HPP

#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/io.hpp>
#include <cctype>
#include "../Utils.hpp"
#include "./../../include/gl_helper.hpp"

#include "./../../include/ShaderProgram.hpp"
#include "./../../include/KeyframedHierarchicalRenderable.hpp"
#include <iostream>

// This class inheriting from KeyframedHierarchicalRenderable
// is actually a hack to have access to do_animate method
// and keyframed / hierarchical behaviors.
class Light : public KeyframedHierarchicalRenderable
{
    public:
    typedef std::shared_ptr<Light> LightPtr; /*!< Smart pointer to a light */

    static glm::vec3 base_forward;
    /**
    * @brief Destructor
    */
    virtual ~Light() 
    {}

    /**
    * @brief Default constructor
    * @param ambient The ambient intensity of the light.
    * @param diffuse The diffuse intensity of the light.
    * @param specular The specular intensity of the light.
    */
    Light(const glm::vec3 & ambient, const glm::vec3 & diffuse, const glm::vec3 & specular):
        KeyframedHierarchicalRenderable(),
        m_ambient(ambient), m_diffuse(diffuse), m_specular(specular)
    {}

    /**
    * @brief Get location for the attributes of the light and send the data to the GPU as uniforms.
    *
    * @param program A pointer to the shader program where to get the locations.
    * @param light A pointer to the light to send to the GPU.
    * @return  True if everything was fine, false otherwise
    */
    static bool sendToGPU(const ShaderProgramPtr& program, const LightPtr & light);
    
    /**
    * @brief Get location for the attributes of the lights and send the data to the GPU as uniforms.
    *
    * @param program A pointer to the shader program where to get the locations.
    * @param lights A vector of pointer to the lights to send to the GPU.
    * @return  True if everything was fine, false otherwise
    */
    template<typename T>
    static bool sendToGPU(const ShaderProgramPtr& program, const std::vector<std::shared_ptr<T>> & lights)
    {
        static_assert(std::is_base_of<Light, T>::value, "The type of light should inherit from Light (see Light.hpp)");
        static_assert(!std::is_same<Light, T>::value, "The type of light should not be Light (see Light.hpp)");
        
        if (program == nullptr || lights.empty())
            return false;

        bool success = true;
        LightPtr first = lights[0];
        std::string light_name = first->lightName();
        std::string type_name = light_name;
        type_name[0] = std::toupper(type_name[0]);
        int location = program->getUniformLocation("numberOf" + type_name);
        
        if(location!=ShaderProgram::null_location){
            glcheck(glUniform1i(location, (int)lights.size()));
        } else { success = false; }
        
        for(size_t i=0; i<lights.size(); ++i){
            std::string name_with_index = first->lightName() + "[" + std::to_string(i) + "]";
            LightPtr light = lights[i];
            success &= light->sendToGPU(program, name_with_index);
        }
        return success;
    }

    /**
    * @brief Access to the ambient intensity of the light.
    *
    * @return A const reference to m_ambient.
    */
    const glm::vec3 &ambient() const { return m_ambient; }

    /**
    * @brief Set the ambient intensity of the light.
    *
    * Set the value of m_ambient.
    * @param ambient The new ambient intensity of the light.
    */
    void setAmbient(const glm::vec3 &ambient) { m_ambient=ambient; }

    /**
    * @brief Access to the diffuse intensity of the light.
    *
    * @return A const reference to m_diffuse.
    */
    const glm::vec3& diffuse() const { return m_diffuse; }

    /**
    * @brief Set the diffuse intensity of the light.
    *
    * Set the value of m_diffuse.
    * @param diffuse The new diffuse intensity of the light.
    */
    void setDiffuse(const glm::vec3 &diffuse) { m_diffuse=diffuse; }

    /**
    * @brief Access to the specular intensity of the light.
    *
    * @return A const reference to m_specular.
    */
    const glm::vec3 &specular() const { return m_specular; }

    /**
    * @brief Set the specular intensity of the light.
    *
    * Set the value of m_specular.
    * @param specular The new specular intensity of the light.
    */
    void setSpecular(const glm::vec3 &specular) { m_specular=specular; }
    
    /** 
    * @brief Get the uniform name of the light.
    *
    * @return The name of the light in the shader.
    */
    virtual std::string lightName() const =0;
    
    protected:
    void do_animate(float time){
        KeyframedHierarchicalRenderable::do_animate(time);
        updateModelMatrix();
    }

    virtual bool sendToGPU(const ShaderProgramPtr& program, const std::string & identifier)const =0;
    
    private:
    void do_draw()
    {}

    glm::vec3 m_ambient;    /*!< Intensity of the light with respect to the object ambient components. */
    glm::vec3 m_diffuse;    /*!< Intensity of the light with respect to the object diffuse components. */
    glm::vec3 m_specular;   /*!< Intensity of the light with respect to the object specular components. */
};

typedef std::shared_ptr<Light> LightPtr; /*!< Smart pointer to a light */


/**
 * @brief A directional light.
 *
 * This class represents a directional light which means a light
 * whose source is so far away from the camera that the light rays
 * can be considered close to parallel from each other regardless of
 * the position of the objects or the camera. A good example is the
 * sun.
 */
class DirectionalLight : public Light
{
    public:
    /**
     * @brief Destructor
     */
    virtual ~DirectionalLight() {}

    /**
    * @brief Specific constructor
    *
    * Construct a directional light.
    *
    * @param direction The direction of the light.
    */
    DirectionalLight(const glm::vec3 & direction,
                     const glm::vec3 & ambient,
                     const glm::vec3 & diffuse,
                     const glm::vec3 & specular):
        Light(ambient, diffuse, specular), m_direction(direction)
    {
        setGlobalTransform(lookAtModel(glm::vec3(0), m_direction, Light::base_forward));
    }
    
    /**
     * @brief Access to the direction of the light.
     *
     * @return A const reference to m_direction.
     */
    const glm::vec3& direction() const { return m_direction; }

    /**
     * @brief Set the direction of the light.
     *
     * Set the value of m_direction.
     * @param direction The new direction of the light.
     */
    void setDirection(const glm::vec3 &direction) { m_direction=direction; }

    protected:
    void do_animate(float time){
        Light::do_animate(time);
        glm::mat4 model = getModelMatrix();
        glm::vec3 scale, skew, translation;
        glm::quat rotation;
        glm::vec4 perspective;
        glm::decompose(model, scale, rotation, translation, skew, perspective);
        m_direction = glm::conjugate(rotation) * Light::base_forward;
    }

    private:
    /**
     * @brief Get the uniform name of the light.
     *
     * @return The name of the light in the shader.
     */
    std::string lightName() const { return "directionalLight"; }
    bool sendToGPU(const ShaderProgramPtr& program, const std::string & identifier) const;

    glm::vec3 m_direction;  /*!< The direction of the light. */
};

typedef std::shared_ptr<DirectionalLight> DirectionalLightPtr; /*!< Smart pointer to a directional light */


class PointLight : public Light
{
    public:
    virtual ~PointLight()
    {}

    /**
     * @brief Specific constructor
     *
     * Construct a point light.
     *
     * @param position The position of the light.
     * @param ambient The ambient intensity of the light.
     * @param diffuse The diffuse intensity of the light.
     * @param specular The specular intensity of the light.
     * @param constant The coefficient of constant attenuation of the light.
     * @param linear The coefficient of linear attenuation of the light with respect to the distance to the light.
     * @param quadratic The coefficient of quadratic attenuation of the light with respect to the distance to the light.
     */
    PointLight(const glm::vec3 & position,
            const glm::vec3 & ambient,
            const glm::vec3 & diffuse,
            const glm::vec3 & specular,
            float constant,
            float linear,
            float quadratic):
        Light(ambient, diffuse, specular), m_position(position),
        m_constant(constant), m_linear(linear), m_quadratic(quadratic)
    {
        setGlobalTransform(getTranslationMatrix(m_position));
    }

    /**
     * @brief Access to the position of the light.
     *
     * @return A const reference to m_position.
     */
    const glm::vec3 &position() const { return m_position; }

    /**
     * @brief Set the position of the light.
     *
     * Set the value of m_position.
     * @param position The new position of the light.
     */
    void setPosition(const glm::vec3 &position) { m_position=position; }

    /**
     * @brief Access to the coefficient of constant attenuation of the light.
     *
     * @return A const reference to m_constant.
     */
    const float &constant() const { return m_constant; }

    /**
     * @brief Set the coefficient of constant attenuation of the light.
     *
     * Set the value of m_constant.
     * @param constant The new coefficient of constant attenuation of the light.
     */
    void setConstant(float constant) { m_constant=constant; }

    /**
     * @brief Access to the coefficient of linear attenuation of the light.
     *
     * @return A const reference to m_linear.
     */
    const float& linear() const { return m_linear; }

    /**
     * @brief Set the coefficient of linear attenuation of the light.
     *
     * Set the value of m_linear.
     * @param linear The new coefficient of linear attenuation of the light.
     */
    void setLinear(float linear) { m_linear=linear; }

    /**
     * @brief Access to the coefficient of quadratic attenuation of the light.
     *
     * @return A const reference to m_quadratic.
     */
    const float &quadratic() const { return m_linear; }

    /**
     * @brief Set the coefficient of quadratic attenuation of the light.
     *
     * Set the value of m_quadratic.
     * @param quadratic The new coefficient of quadratic attenuation of the light.
     */
    void setQuadratic(float quadratic) { m_quadratic=quadratic; }
    
    protected:
    void do_animate(float time){
        Light::do_animate(time);
        glm::mat4 model = getModelMatrix();
        for (size_t i=0u;i<3;++i)
            m_position[i] = model[3][i];
    }
    bool sendToGPU(const ShaderProgramPtr& program, const std::string & identifier) const;

    glm::vec3 m_position;   /*!< The position of the light. */

    private:
    std::string lightName() const { return "pointLight"; }
    
    float m_constant;       /*!< Coefficient of constant attenuation of the light. */
    float m_linear;         /*!< Coefficient of linear attenuation of the light with respect to the distance to the light position. */
    float m_quadratic;      /*!< Coefficient of quadratic attenuation of the light with respect to the distance to the light position. */
};

typedef std::shared_ptr<PointLight> PointLightPtr; /*!< Smart pointer to a point light */

/**
 * @brief A spot light.
 *
 * This class represents a spot light which means a light
 * whose source is located somewhere in the environment and
 * instead of shooting light rays in all directions,
 * only shoots them in a specific direction.
 * As a result, only the objects within a certain radius
 * of the spotlight's direction are lit and everything else stays dark.
 * A good example of a spotlight would be a street lamp.
 */
class SpotLight : public PointLight
{
    public:
    /**
     * @brief Destructor
     */
    virtual ~SpotLight()
    {}

    /**
    * @brief Specific constructor
    *
    * Construct a spot light.
    *
    * @param position The position of the light.
    * @param spotDirection The direction of the light.
    * @param ambient The ambient intensity of the light.
    * @param diffuse The diffuse intensity of the light.
    * @param specular The specular intensity of the light.
    * @param constant The coefficient of constant attenuation of the light.
    * @param linear The coefficient of linear attenuation of the light with respect to the distance to the light.
    * @param quadratic The coefficient of quadratic attenuation of the light with respect to the distance to the light.
    * @param innerCutOff The cosinus of the inner cut off angle of the spot.
    * @param outerCutOff The cosinus of the outer cut off angle of the spot.
    */
    SpotLight(const glm::vec3& position, const glm::vec3& spotDirection,
              const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular,
              float constant, float linear, float quadratic,
              float innerCutOff, float outerCutOff):
        PointLight(position, ambient, diffuse, specular, constant, linear, quadratic),
        m_spotDirection(spotDirection), m_innerCutOff(innerCutOff), m_outerCutOff(outerCutOff)
    {
        setGlobalTransform(lookAtModel(m_position, m_spotDirection, Light::base_forward));
        updateModelMatrix();
    }

    /**
     * @brief Access to the direction of the spot.
     *
     * @return A const reference to m_direction.
     */
    const glm::vec3 &spotDirection() const { return m_spotDirection; }

    /**
     * @brief Set the direction of the spot.
     *
     * Set the value of m_direction.
     * @param spotDirection The new direction of the light.
     */
    void setSpotDirection(const glm::vec3 &spotDirection) { m_spotDirection=spotDirection; }

    /**
     * @brief Access to the cosinus of the inner cut off angle of the spot.
     *
     * @return A const reference to m_innerCutOff.
     */
    float innerCutOff() const { return m_innerCutOff; }

    /**
     * @brief Set the cosinus of the inner cut off angle of the spot.
     *
     * Set the value of m_innerCutOff.
     * @param innerCutOff The new cosinus of the inner cut off angle of the spot.
     */
    void setInnerCutOff(float innerCutOff) { m_innerCutOff=innerCutOff; }

    /**
     * @brief Access to the cosinus of the outer cut off angle of the spot.
     *
     * @return A const reference to m_outerCutOff.
     */
    float outerCutOff() const { return m_innerCutOff; }

    /**
     * @brief Set the cosinus of the outer cut off angle of the spot.
     *
     * Set the value of m_outerCutOff.
     * @param outerCutOff The new cosinus of the outer cut off angle of the spot.
     */
    void setOuterCutOff(float outerCutOff) { m_outerCutOff=outerCutOff; }
    
    protected:
    void do_animate(float time){
        Light::do_animate(time);
        glm::mat4 model = getGlobalTransform();//getModelMatrix();
        glm::vec3 scale, skew, translation;
        glm::quat rotation;
        glm::vec4 perspective;
        glm::decompose(model, scale, rotation, translation, skew, perspective);
        m_position = translation;
        m_spotDirection = glm::conjugate(rotation) * Light::base_forward;
    }

    private:
    /**
     * @brief Get the uniform name of the light.
     *
     * @return The name of the light in the shader.
     */
    std::string lightName() const { return "spotLight"; }
    bool sendToGPU(const ShaderProgramPtr& program, const std::string & identifier) const;
    glm::vec3 m_spotDirection; /*!< The direction of the spot. */
    float m_innerCutOff;    /*!< The cosinus of the inner cutoff angle that specifies the spotlight's inner radius. Everything inside this angle is fully lit by the spotlight. */
    float m_outerCutOff;    /*!< The cosinus of the outer cutoff angle that specifies the spotlight's outer radius. Everything outside this angle is not lit by the spotlight. */
};

typedef std::shared_ptr<SpotLight> SpotLightPtr; /*!< Smart pointer to a spot light */

#endif //LIGHT_HPP

#include "./../../include/dynamics/ControlledForceFieldRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>

ControlledForceFieldStatus::~ControlledForceFieldStatus(){}

ControlledForceFieldStatus::ControlledForceFieldStatus()
{
    clear();
}

ControlledForceFieldStatus::ControlledForceFieldStatus(const glm::vec3& initial_direction)
{
    clear();
    initial =  initial_direction;
    movement = initial_direction;
};

void ControlledForceFieldStatus::clear()
{
    initial = glm::vec3(0,0,0);
    movement = glm::vec3(0,0,0);
    angle =  0;
    last_time =  0;
    intensity = 0;
    acceleration = 10.0;
    deacceleration = 5.0;
    angularSpeed = 2.0;
    dampingFactor = 0.8;
    min_intensity = -1;
    max_intensity = 10;

    accelerating =  false;
    deaccelerating =  false;
    turning_left =  false;
    turning_right =  false;
}

void ControlledForceFieldStatus::compute_movement(){
    float cos = std::cos( angle );
    float sin = std::sin( angle );
    movement = glm::vec3(cos * initial.x - sin * initial.z,
                         0,
                         sin * initial.x + cos * initial.z);
}

ControlledForceFieldRenderable::~ControlledForceFieldRenderable()
{}

ControlledForceFieldRenderable::ControlledForceFieldRenderable(ShaderProgramPtr program,ConstantForceFieldPtr forceField )
    : ConstantForceFieldRenderable(program, forceField)
{
    glm::vec3 initial_direction(0,0,1);
    m_status = ControlledForceFieldStatus(initial_direction);
}

void ControlledForceFieldRenderable::do_keyPressedEvent( sf::Event& e )
{
    if( e.key.code == sf::Keyboard::Left )
    {
        m_status.turning_left = true;
    }
    else if( e.key.code == sf::Keyboard::Right )
    {
        m_status.turning_right = true;
    }
    else if( e.key.code == sf::Keyboard::Up )
    {
        m_status.accelerating = true;
    }
    else if( e.key.code == sf::Keyboard::Down )
    {
        m_status.deaccelerating = true;
    }
}

void ControlledForceFieldRenderable::do_keyReleasedEvent( sf::Event& e )
{
    if( e.key.code == sf::Keyboard::Left )
    {
        m_status.turning_left = false;
    }
    else if( e.key.code == sf::Keyboard::Right )
    {
        m_status.turning_right = false;
    }
    else if( e.key.code == sf::Keyboard::Up )
    {
        m_status.accelerating = false;
    }
    else if( e.key.code == sf::Keyboard::Down )
    {
        m_status.deaccelerating = false;
    }
}

void ControlledForceFieldRenderable::do_animate( float time )
{
    if( time > m_status.last_time )
    {
        float dt = time - m_status.last_time;

        if ( m_status.turning_left && !m_status.turning_right )
        {
            m_status.angle -= dt * m_status.angularSpeed;
            m_status.compute_movement();
        }
        else if( m_status.turning_right && !m_status.turning_left )
        {
            m_status.angle += dt * m_status.angularSpeed;
            m_status.compute_movement();
        }

        if( m_status.accelerating )
            m_status.intensity += dt * m_status.acceleration;
        else if( m_status.deaccelerating )
            m_status.intensity -= dt * m_status.deacceleration;
        else m_status.intensity *= dt * m_status.dampingFactor;

        m_status.intensity = glm::clamp( m_status.intensity, m_status.min_intensity, m_status.max_intensity );

        m_forceField->setForce( m_status.movement * m_status.intensity );
    }
    m_status.last_time = time;
}

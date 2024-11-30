#include "./../include/Camera.hpp"
#include "./../include/log.hpp"
#include <glm/gtc/type_precision.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/vector_angle.hpp> 
#include <glm/gtx/io.hpp> 
#include <limits>

using namespace std;

static constexpr std::array<const char *, Camera::CAMERA_BEHAVIOR::BEHAVIOR_NUMBER> CAMERA_BEHAVIOR_NAMES =
    {"FirstPerson", "Arcball", "Trackball", "Spaceship"};

const glm::vec3 Camera::base_up = glm::vec3(0, 1, 0);
const glm::vec3 Camera::base_forward = glm::vec3(0, 0, -1);

Camera::Camera()
    : m_view{ glm::lookAt( glm::vec3{0, 0, -5}, glm::vec3{}, glm::vec3{0,1,0}) },
      m_fov{ M_PI * 0.5 }, m_ratio{ 1.0f }, m_znear{ 0.01f }, m_zfar{ 250.0f },
      m_behavior{ FIRST_PERSON_BEHAVIOR }, m_last_q{1.0f, 0.0f, 0.0f, 0.0f},
      m_current_q{1.0f, 0.0f, 0.0f, 0.0f}
{
    updateGlobalTransform();
    updateModelMatrix();
    m_base_view = m_view;
}

Camera::~Camera()
{}

void Camera::do_animate(float time)
{
    KeyframedHierarchicalRenderable::do_animate(time);
    updateModelMatrix();
    m_view = glm::inverse(getModelMatrix());
}

const glm::mat4& Camera::viewMatrix() const
{
    return m_view;
}

void Camera::updateGlobalTransform(){
    setGlobalTransform(glm::inverse(m_view));
}

void Camera::setViewMatrix(const glm::mat4& view)
{
    m_view = view;
    updateGlobalTransform();
    updateModelMatrix();
}

glm::vec3 Camera::getPosition() const
{
    return -glm::vec3( m_view[3] ) * glm::mat3( m_view );
}

glm::vec3 Camera::getRight() const
{
    return glm::vec3( m_view[0][0], m_view[1][0], m_view[2][0] );
}

glm::vec3 Camera::getUp() const
{
    return glm::vec3( m_view[0][1], m_view[1][1], m_view[2][1] );
}

glm::vec3 Camera::getForward() const
{
    return glm::vec3( -m_view[0][2], -m_view[1][2], -m_view[2][2] );
}

void Camera::setPosition( const glm::vec3& pos )
{
    for( int i = 0; i < 3; ++ i )
    {
        m_view[3][i] = float(0);
        for( int j = 0; j < 3; ++ j )
        {
            m_view[3][i] -= m_view[j][i] * pos[j];
        }
    }
}

void Camera::setRight( const glm::vec3& right )
{
    glm::vec3 minus_pos = glm::vec3( m_view[3] ) * glm::mat3( m_view );
    m_view[0][0] = right.x;
    m_view[1][0] = right.y;
    m_view[2][0] = right.z;
    m_view[3][0] = dot( right, minus_pos );
}

void Camera::setUp( const glm::vec3& up )
{
  glm::vec3 minus_pos = glm::vec3( m_view[3] ) * glm::mat3( m_view );
  m_view[0][1] = up.x;
  m_view[1][1] = up.y;
  m_view[2][1] = up.z;
  m_view[3][1] = dot( up, minus_pos );
}

void Camera::setForward( const glm::vec3& forward )
{
  glm::vec3 pos = -glm::vec3( m_view[3] ) * glm::mat3( m_view );
  m_view[0][0] = -forward.x;
  m_view[1][0] = -forward.y;
  m_view[2][0] = -forward.z;
  m_view[3][0] = dot( forward, pos );
}

const glm::mat4& Camera::projectionMatrix() const
{
    return m_projection;
}

void Camera::setProjectionMatrix(const glm::mat4& projection)
{
    m_projection = projection;
}

float Camera::fov() const
{
    return m_fov;
}

float Camera::ratio() const
{
    return m_ratio;
}

float Camera::znear() const
{
    return m_znear;
}

float Camera::zfar() const
{
    return m_zfar;
}

void Camera::setFov( const float& v )
{
    m_fov = v;
    m_projection = glm::perspective( m_fov, m_ratio, m_znear, m_zfar );
}

void Camera::setRatio( const float& v )
{
    m_ratio = v;
    m_projection = glm::perspective( m_fov, m_ratio, m_znear, m_zfar );
}

void Camera::setZfar( const float& v )
{
    m_zfar = v;
    m_projection = glm::perspective( m_fov, m_ratio, m_znear, m_zfar );
}

void Camera::setZnear( const float& v )
{
    m_znear = v;
    m_projection = glm::perspective( m_fov, m_ratio, m_znear, m_zfar );
}

Camera::CAMERA_BEHAVIOR Camera::getBehavior() const
{
    return m_behavior;
}

void Camera::incrementBehavior()
{
    m_behavior = CAMERA_BEHAVIOR((int(m_behavior) + 1) % BEHAVIOR_NUMBER);
    setBehavior(m_behavior);
}


void Camera::setBehavior(CAMERA_BEHAVIOR behavior )
{
    m_behavior = behavior;
    if( m_behavior == ARCBALL_BEHAVIOR
        || m_behavior == TRACKBALL_BEHAVIOR )
    {
        glm::vec3 pos = getPosition();
        glm::vec3 up = getUp();
        m_view = glm::lookAt(
            getPosition(),
            glm::vec3{},
            getUp());
    }
    if (m_behavior == ARCBALL_BEHAVIOR){
        m_start = glm::vec2(0.0f);
        m_current = glm::vec2(0.0f);
        m_current_q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 pos = getPosition();
        glm::vec3 base_pos = -glm::vec3( m_base_view[3] ) * glm::mat3( m_base_view );
        float angle = glm::angle(base_pos, pos);
        glm::vec3 axis = glm::cross(base_pos, pos);
        // Just a safety check
        if (glm::length(axis) < std::numeric_limits<float>::epsilon())
            axis = glm::vec3(1,0,0);
        else
            axis = glm::normalize(axis);
        m_last_q = glm::angleAxis(angle, axis);
    } 
    LOG( info, "Camera changed to " << CAMERA_BEHAVIOR_NAMES[m_behavior] << ".");
}

glm::vec3 Camera::project(glm::vec2 p) 
{
    float r = 1.0f;
    p.x = p.x * m_ratio;
    float z;
    float x2 = p.x * p.x;
    float y2 = p.y * p.y;
    float r2 = r * r;
    if (x2 + y2 <= r2 * 0.5)
      z = std::sqrt(r2 - x2 - y2);
    else
      z = (r2 * 0.5) / std::sqrt(x2 + y2);
    return glm::vec3(p, z);
}

void Camera::mousePress(glm::vec2 pos){ 
    m_start = pos;
    m_current = pos;
}
void Camera::mouseRelease(){ 
    m_last_q = m_last_q * m_current_q;
    m_start = glm::vec2(0.0f);
    m_current = m_start;
}

void Camera::update(glm::vec2 dpos)
{
    switch( m_behavior )
    {
    case TRACKBALL_BEHAVIOR:
    {
        glm::mat4 rotation = glm::rotate( glm::mat4( glm::mat3(m_view) ), dpos.x, getUp() );
        rotation = glm::rotate( rotation, dpos.y, getRight() );

        m_view[0] = rotation[0];
        m_view[1] = rotation[1];
        m_view[2] = rotation[2];

        setPosition( -glm::vec3(m_view[3]) * glm::mat3(rotation) );
    }
        break;
    case ARCBALL_BEHAVIOR:
    {
        m_current += dpos;
        glm::vec3 a = glm::normalize(project(m_start));
        glm::vec3 b = glm::normalize(project(m_current));
        float angle = glm::angle(a, b);
        if (std::abs(angle) > std::numeric_limits<float>::epsilon())
        {
            glm::vec3 axis = glm::inverse(m_last_q) * glm::normalize(glm::cross(a, b));
            m_current_q = glm::angleAxis(angle, axis);
            glm::quat rot_q = m_last_q * m_current_q;

            glm::mat4 rotation = glm::rotate( glm::mat4( glm::mat3(m_base_view) ), glm::angle(rot_q), glm::axis(rot_q) );

            m_view[0] = rotation[0];
            m_view[1] = rotation[1];
            m_view[2] = rotation[2];

            glm::vec3 dir = glm::normalize(glm::vec3(m_base_view[3]));
            float length = glm::length(glm::vec3(m_view[3]));
            
            setPosition( (-dir * length) * glm::mat3(rotation) );

        }
    }
        break;

    case SPACESHIP_BEHAVIOR:
    {
        const float cx = float( std::cos( dpos.x ) );
        const float sx = float( std::sin( dpos.x ) );
        const float cy = float( std::cos( dpos.y ) );
        const float sy = float( std::sin( dpos.y ) );

        glm::mat3 rotation = glm::mat3( m_view );
        glm::vec3 minus_pos = glm::vec3( m_view[3] ) * rotation;
        // build a rotation matrix to apply to the current rotation:
        rotation = glm::mat3(
                    glm::vec3( cx, sx*sy,-sx*cy),
                    glm::vec3(  0,    cy,    sy),
                    glm::vec3( sx,-cx*sy, cx*cy) ) * rotation;
        for( int col = 0; col < 3; ++ col )
        {
            for( int lin = 0; lin < 3; ++ lin )
            {
                m_view[col][lin] = rotation[col][lin];
            }
        }
        m_view[3][0] = dot( minus_pos, getRight() );
        m_view[3][1] = dot( minus_pos, getUp() );
        m_view[3][2] = dot( minus_pos, -getForward() );
    }
        break;

    case FIRST_PERSON_BEHAVIOR:
        break;

    default:
        assert(false);
    }
    updateGlobalTransform();
    updateModelMatrix();
}

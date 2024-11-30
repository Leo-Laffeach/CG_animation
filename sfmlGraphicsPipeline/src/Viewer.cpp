#include "./../include/Viewer.hpp"
#include "./../include/gl_helper.hpp"
#include "./../include/log.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/norm.hpp>
#include <GL/glew.h>
#include <iostream>
#include <sstream>
#include <iomanip>

bool PriorityComparator::operator()(const RenderablePtr & a, const RenderablePtr & b) const
{
    return a->priority() > b->priority(); 
}

static const Viewer::Duration g_modeInformationTextTimeout = std::chrono::seconds( 3 );

static const std::string screenshot_basename = "screenshot";

static void initializeGL()
{
    //Initialize GLEW
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if( GLEW_OK != err )
        LOG( error, "[GLEW] " << glewGetErrorString(err) );
    LOG( info, "[GLEW] using version " << glewGetString( GLEW_VERSION ) );
}

Viewer::KeyboardState::KeyboardState()
    : speed( 3.0f ), orientation(0.0f), 
      aspeed(2.0f), direction(0.0f)
{}

glm::vec3 Viewer::KeyboardState::normalized_direction(){
    if(glm::any(glm::bvec3(direction)))
        return glm::normalize(direction);
    return direction;
}

Viewer::~Viewer()
{}

Viewer::Viewer(float width, float height, const glm::vec4 & background_color) :
    m_window{
        sf::VideoMode(width, height),
        "Computer Graphics Practicals",
        sf::Style::Default,
        sf::ContextSettings{ 24 /* depth*/, 8 /*stencil*/, 4 /*anti aliasing level*/, 4 /*GL major version*/, 0 /*GL minor version*/}
    },
    //m_modeInformationTextDisappearanceTime{ clock::now() + g_modeInformationTextTimeout },
    //m_modeInformationText{ "Arcball Camera Activated" },
    m_applicationRunning{ true }, m_animationLoop{ false }, m_animationIsStarted{ false },
    m_loopDuration{120}, m_simulationTime{0},
    m_screenshotCounter{0}, m_helpDisplayed{false}, m_helpDisplayRequest{false},
    m_lastEventHandleTime{ clock::now() },
    m_background_color{background_color}   
{   
    sf::ContextSettings settings = m_window.getSettings();
    LOG( info, "Settings of OPENGL Context created by SFML");
    LOG( info, "\tdepth bits:         " << settings.depthBits );
    LOG( info, "\tstencil bits:       " << settings.stencilBits );
    LOG( info, "\tantialiasing level: " << settings.antialiasingLevel);
    LOG( info, "\tGL version:         " << glGetString( GL_VERSION ) );
    LOG( info, "\tGL renderer:        " << glGetString( GL_RENDERER ) );

    //Initialize the camera
    float ratio = width/height;
    m_camera.setRatio(ratio);
    //Set up GLEW
    initializeGL();
    //Initialize OpenGL context
    setBackgroundColor(m_background_color);
    glcheck(glEnable(GL_DEPTH_TEST));
    glcheck(glDepthFunc(GL_LESS));
    glcheck(glEnable(GL_VERTEX_PROGRAM_POINT_SIZE));
    glcheck(glEnable(GL_TEXTURE_2D));

    m_texture.create(width, height, sf::ContextSettings{ 0 /* depth*/, 0 /*stencil*/, 4 /*anti aliasing level*/, 4 /*GL major version*/, 0 /*GL minor version*/});
    //Initialize the text engine (this SHOULD be done after initializeGL, as the text
    //engine store some data on the graphic card)
    //m_tengine.init();
    //m_tengine.setWindowDimensions( m_window.getSize().x, m_window.getSize().y );
}

constexpr const char * g_help_message =
        "VIEWER SHORTCUTS:\n"
        "      [F1]  Display/Hide this help message\n"
        "      [F2]  Take a screen shot of the frame currently in the frame buffer\n"
        "      [F3]  Reload all managed shader program from their sources\n"
        "      [F4]  Pause/Stop the animation\n"
        "      [F5]  Reset the animation\n"
        "       [c]  Switch the camera mode between First Person / Arcball / Trackball / Space ship\n"
        "[ctrl]+[w]  Quit the application\n"
        "\n"
        "CAMERA CONTROL:\n"
        "  * First Person mode\n"
        "    [q] to go left         [d] to go right\n"
        "    [z] to go forward      [z] to go backward\n"
        "    [up] look up           [down] look down\n"
        "    [left] look left       [look right] look right\n"
        "    [LShift] to go down  [Space] to go up\n"
        "\n"
        "  * Arcball and Trackball modes\n"
        "    [mouse rclick] to rotate the camera around the origin\n"
        "    [mouse wheel] to change the zoom\n"
        "    Note : The arcball camera is invariant to the path of the mouse, not the trackball camera.\n"
        "\n"
        "  * Spaceship mode\n"
        "    [mouse rclick] to rotate the camera around itself\n"
        "    [q] to go left         [d] to go right\n"
        "    [z] to go forward      [z] to go backward\n"
        "    [LShift] to go slower  [Space] to go faster\n"
        "\n"
        "Note: none of those shortcut keyboard events are transmitted to other renderables."
        ;



void Viewer::draw()
{
    glcheck(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
    float time = getTime();
    for( const ShaderProgramPtr & prog : m_programs )
    {
        prog->bind();

        Light::sendToGPU<DirectionalLight>( prog, m_directionalLights);
        Light::sendToGPU<SpotLight>( prog, m_spotLights);
        Light::sendToGPU<PointLight>( prog, m_pointLights);

        int timeLocation = prog->getUniformLocation("time");
        if(timeLocation != ShaderProgram::null_location)
            glcheck(glUniform1f(timeLocation, time));
    }

    for(const RenderablePtr & r : m_renderables)
    {   
        if( r->getShaderProgram() )
        {
            r->bindShaderProgram();
            int projectionLocation = r->projectionLocation();
            if(projectionLocation != ShaderProgram::null_location)
                glcheck(glUniformMatrix4fv(projectionLocation, 1, GL_FALSE, glm::value_ptr(m_camera.projectionMatrix())));
            
            int viewLocation = r->viewLocation();
            if(viewLocation != ShaderProgram::null_location)
                glcheck(glUniformMatrix4fv(viewLocation, 1, GL_FALSE, glm::value_ptr(m_camera.viewMatrix())));

            // Texture 
            int texsamplerLocation = r->getShaderProgram()->getUniformLocation("ViewerTexSampler");
            if (texsamplerLocation != ShaderProgram::null_location)
            {   
                glEnable(GL_TEXTURE_2D);
                glActiveTexture(GL_TEXTURE0);
                sf::Texture::bind(&(m_texture.getTexture()));
                glUniform1i(texsamplerLocation, 0) ;
            }
            
        }
        if(r->getRenderMode() <= Renderable::RENDER_MODE::WINDOW_TEXTURE)
        {
            r->draw();
        }  
        if(r->getRenderMode() >= Renderable::RENDER_MODE::WINDOW_TEXTURE)
        {
            m_texture.setActive(true);
            r->draw();
            m_texture.display();
            m_texture.setActive(false);
        }
        if(r->getShaderProgram())
        {
            sf::Texture::bind(0);
            glDisable(GL_TEXTURE_2D);
        }
        r->unbindShaderProgram();
    }

    if (m_helpDisplayRequest && !m_helpDisplayed){
        LOG(info, g_help_message);
        m_helpDisplayed = true;
    }
    //Refresh the viewer.m_window
    /*
    if( clock::now() < m_modeInformationTextDisappearanceTime )
    {
        //m_tengine.render( m_modeInformationText, glm::vec2(10, m_window.getSize().y - 30), glm::vec3(0.1, 0.1, 0.1) );
    }
    {
        std::ostringstream ss;
        ss << "FPS: " << std::setprecision( 2 ) << std::fixed << m_fpsCounter.getFPS();
        //m_tengine.render( ss.str(), glm::vec2(m_window.getSize().x - 200, m_window.getSize().y - 30), glm::vec3(0.1,0.1,0.1) );
    }
    if( m_helpDisplayed )
        m_tengine.render( g_help_message, glm::vec2(100, 650), glm::vec3{.0, .1, .2});
    */

}

float Viewer::getTime()
{
    if( m_animationIsStarted )
    {
        m_simulationTime += Duration( clock::now() - m_lastSimulationTimePoint).count();
        m_lastSimulationTimePoint = clock::now();
    }
    if( m_animationLoop && m_simulationTime >= m_loopDuration )
        m_simulationTime = std::fmod( m_simulationTime, m_loopDuration );
    return m_simulationTime;
}

void Viewer::animate()
{
    if(m_animationIsStarted)
    {
        for(const RenderablePtr & r : m_renderables)
            r->animate( getTime() );
        for (const DirectionalLightPtr & dl : m_directionalLights)
            dl->animate( getTime() );
        for (const PointLightPtr & pl : m_pointLights)
            pl->animate( getTime() );
        for (const SpotLightPtr & sl : m_spotLights)
            sl->animate( getTime() );

        m_camera.animate( getTime() );
    }
}

void Viewer::setAnimationLoop(bool animationLoop, float loopDuration)
{
    m_animationLoop = animationLoop;
    m_loopDuration = loopDuration;
}

void Viewer::addDirectionalLight(const DirectionalLightPtr& directionalLight)
{
    m_directionalLights.push_back(directionalLight);
}

void Viewer::addPointLight(const PointLightPtr& pointLight)
{
    m_pointLights.push_back(pointLight);
}

void Viewer::addSpotLight(const SpotLightPtr& spotLight)
{
    m_spotLights.push_back(spotLight);
}

void Viewer::startAnimation()
{
    m_lastSimulationTimePoint = clock::now();
    m_animationIsStarted=true;
}

void Viewer::stopAnimation()
{
    m_animationIsStarted=false;
}

void Viewer::resetAnimation()
{
    m_lastSimulationTimePoint = clock::now();
    m_simulationTime = 0;
}



void Viewer::addRenderable(const RenderablePtr & r)
{   
    r->m_viewer = this;
    m_renderables.insert(r);
}

void Viewer::keyPressedEvent(sf::Event& e)
{
    switch (e.key.code)
    {
    case sf::Keyboard::C:   
        changeCameraMode();
        break;
    case sf::Keyboard::R:
        m_renderables.clear();
        LOG(info, "Renderables cleared.")
        break;
    case sf::Keyboard::F1:
        m_helpDisplayRequest = true;
        break;
    case sf::Keyboard::F2:
        takeScreenshot();
        break;
    case sf::Keyboard::F3:
        reloadShaderPrograms();
        LOG(info, "Shaders reloaded.")
        break;
    case sf::Keyboard::F4:
        if( m_animationIsStarted )
        {
            stopAnimation();
            LOG(info, "Animation paused.")
        }
        else
        {
            startAnimation();
            LOG(info, "Animation resumed.")
        } 
        break;
    case sf::Keyboard::F5:
        resetAnimation();
        for(const RenderablePtr & r : m_renderables)
            r->keyPressedEvent(e);
        LOG(info, "Animation reset.")
        break;
    case sf::Keyboard::W:
        if( e.key.control )
            m_applicationRunning = false;
        break;

    case sf::Keyboard::Q:
        m_keyboard.direction.x = -1;
        break;

    case sf::Keyboard::S:
        m_keyboard.direction.z = -1;
        break;

    case sf::Keyboard::D:
        m_keyboard.direction.x = 1;
        break;

    case sf::Keyboard::Z:
        m_keyboard.direction.z = 1;
        break;

    case sf::Keyboard::Up:
        m_keyboard.orientation.x = -1;
        break;

    case sf::Keyboard::Down:
        m_keyboard.orientation.x = 1;
        break;

    case sf::Keyboard::Right:
        m_keyboard.orientation.y = 1;
        break;

    case sf::Keyboard::Left:
        m_keyboard.orientation.y = -1;
        break;
    
    case sf::Keyboard::Add:
        m_keyboard.speed *= 1.1;
        LOG(info, "Movement speed = "<<m_keyboard.speed);
        break;
    
    case sf::Keyboard::Subtract:
        m_keyboard.speed /= 1.1;
        LOG(info, "Movement speed  = "<<m_keyboard.speed);
        break;

    case sf::Keyboard::Space:
        m_keyboard.direction.y = 1;
        break;

    case sf::Keyboard::LShift:
        m_keyboard.direction.y = -1;
        break;

    default:
        break;
    }
    for(const RenderablePtr & r : m_renderables)
        r->keyPressedEvent(e);
}

void Viewer::keyReleasedEvent(sf::Event& e)
{
    switch (e.key.code)
    {
    case sf::Keyboard::F1:
        m_helpDisplayed = false;
        m_helpDisplayRequest = false;
        break;
    case sf::Keyboard::Q:
        m_keyboard.direction.x = 0;
        break;

    case sf::Keyboard::S:
        m_keyboard.direction.z = 0;
        break;

    case sf::Keyboard::D:
        m_keyboard.direction.x = 0;
        break;

    case sf::Keyboard::Z:
        m_keyboard.direction.z = 0;
        break;
    case sf::Keyboard::Up:
        m_keyboard.orientation.x = 0;
        break;

    case sf::Keyboard::Down:
        m_keyboard.orientation.x = 0;
        break;

    case sf::Keyboard::Right:
        m_keyboard.orientation.y = 0;
        break;

    case sf::Keyboard::Left:
        m_keyboard.orientation.y = 0;
        break;

    case sf::Keyboard::Space:
        m_keyboard.direction.y = 0;
        break;

    case sf::Keyboard::LShift:
        m_keyboard.direction.y = 0;
        break;
    
    case sf::Keyboard::P:
        LOG(info, "Position  = "<<m_camera.getPosition());
        break;

    case sf::Keyboard::V:
        LOG(info, "Look at = "<<m_camera.getForward());
        break;
    
    case sf::Keyboard::N:
        LOG(info, "Normal = "<<m_camera.getUp());
        break;

    default:
        break;
    }
    for(const RenderablePtr & r : m_renderables)
        r->keyReleasedEvent(e);
}

void Viewer::mousePressEvent(sf::Event& e)
{
    glm::vec2 pos;
    sf::Vector2i pos_pix = sf::Mouse::getPosition(m_window);
    pos.x = 2.0f * pos_pix.x / (float) m_window.getSize().x - 1.0f;
    pos.y = 2.0f * pos_pix.y / (float) m_window.getSize().y - 1.0f;
    m_camera.mousePress(pos);
    for(const RenderablePtr & r : m_renderables)
        r->mousePressEvent(e);
}

void Viewer::mouseReleaseEvent(sf::Event& e)
{   
    m_camera.mouseRelease();
    for(const RenderablePtr & r : m_renderables)
        r->mouseReleaseEvent(e);
}

void Viewer::mouseWheelEvent(sf::Event& e)
{
    // a wheel event modify the distance of the camera to the scene origin
    // when the wheel moved of a distance dz, the camera will be translated of dz * Z_camera
    // note: if we perform this operation inside the camera class, we avoid useless computations.
    // however, it's easier to understand that way.
    if( m_camera.getBehavior() == Camera::ARCBALL_BEHAVIOR
        || m_camera.getBehavior() == Camera::TRACKBALL_BEHAVIOR){
        m_camera.setPosition( m_camera.getPosition() + 0.1f * glm::length(m_camera.getPosition()) * float(e.mouseWheel.delta) * m_camera.getForward() );
        m_camera.updateGlobalTransform();
        m_camera.updateModelMatrix();
    }
    //Solve mouse wheel event for the renderables of the viewer
    for(const RenderablePtr & r : m_renderables)
        r->mouseWheelEvent(e);
}

void Viewer::mouseMoveEvent(sf::Event& e)
{
    //Compute normalized mouse position between [-1,1]
    glm::vec3 normalizedMousePosition;
    normalizedMousePosition.x = 2.0f * e.mouseMove.x / (float)m_window.getSize().x - 1.0f;
    normalizedMousePosition.y = 2.0f * e.mouseMove.y / (float)m_window.getSize().y - 1.0f;
    normalizedMousePosition.z = 1.0f;

    //Set value for current mouse position and displacement from the last move.
    m_currentMousePosition = normalizedMousePosition;
    glm::vec2 deltaMousePosition = glm::vec2(m_currentMousePosition - m_lastMousePosition);

    if( sf::Mouse::isButtonPressed(sf::Mouse::Right) )
    {
        m_camera.update(deltaMousePosition);
    }

    //Set last mouse position.
    m_lastMousePosition = m_currentMousePosition;

    for(const RenderablePtr & r : m_renderables)
        r->mouseMoveEvent(e);
}


void Viewer::handleEvent()
{
    sf::Event event;
    while(m_window.pollEvent(event))
    {
        switch(event.type)
        {
        case sf::Event::Closed:
            m_applicationRunning=false;
            break;
        case sf::Event::Resized:
            m_window.setView(sf::View(sf::FloatRect(0, 0, event.size.width, event.size.height)));
            m_texture.create(event.size.width, event.size.height, sf::ContextSettings{ 0 /* depth*/, 0 /*stencil*/, 4 /*anti aliasing level*/, 4 /*GL major version*/, 0 /*GL minor version*/});
            m_camera.setRatio( (float)(m_window.getSize().x)/(float)(m_window.getSize().y) );
            //m_tengine.setWindowDimensions( m_window.getSize().x, m_window.getSize().y );
            glcheck(glViewport(0, 0, event.size.width, event.size.height));
            break;
        case sf::Event::KeyPressed:
            keyPressedEvent(event);
            break;

        case sf::Event::KeyReleased:
            keyReleasedEvent(event);
            break;
        case sf::Event::MouseWheelMoved:
            mouseWheelEvent(event);
            break;
        case sf::Event::MouseButtonPressed:
            mousePressEvent(event);
            break;
        case sf::Event::MouseButtonReleased:
            mouseReleaseEvent(event);
            break;
        case sf::Event::MouseMoved:
            mouseMoveEvent(event);
            break;
        default:
            break;
        }
    }

    if( m_camera.getBehavior() == Camera::SPACESHIP_BEHAVIOR && glm::any(glm::bvec3(m_keyboard.direction)))
    {
        float speed = m_keyboard.speed * Duration(clock::now() - m_lastEventHandleTime).count();
        glm::vec3 shift = speed * m_keyboard.normalized_direction();
        m_camera.setPosition( m_camera.getPosition()//
                              + shift.x * m_camera.getRight()
                              + shift.y * m_camera.getUp()
                              + shift.z * m_camera.getForward() );
        m_camera.updateGlobalTransform();
        m_camera.updateModelMatrix();
    }
    else if( m_camera.getBehavior() == Camera::FIRST_PERSON_BEHAVIOR)
    {
        float dt = Duration(clock::now() - m_lastEventHandleTime).count();
 
        // Translations

        glm::vec3 step = m_keyboard.speed * dt * m_keyboard.normalized_direction();
        glm::vec3 forward = m_camera.getForward();
        glm::vec3 flat_forward = normalize(forward * glm::vec3(1.0f, 0.0f, 1.0f));
        glm::vec3 flat_right = normalize(m_camera.getRight() * glm::vec3(1.0f, 0.0f, 1.0f));
        glm::vec3 displacement = step.z * flat_forward + step.x * flat_right + step.y * glm::vec3(0.0f, 1.0f, 0.0f);
        
        // Rotations
        glm::vec2 astep = m_keyboard.aspeed * dt;
        // f --> (phi, theta) --> (phi', theta') --> f'
        // f --> (phi, theta)
        float theta = std::acos(forward.y);
        float phi = std::atan2(forward.z, forward.x);
        // (phi, theta) --> (phi', theta')
        theta = std::max(std::min(theta + m_keyboard.orientation.x * astep.x, float(M_PI - 0.01f)), 0.01f);
        phi = std::fmod(phi + m_keyboard.orientation.y * astep.y , 2 * M_PI);
        // (phi', theta') --> f'
        float sintheta = std::sin(theta);
        glm::vec3 new_forward(std::cos(phi) * sintheta, std::cos(theta), std::sin(phi) * sintheta);

        // Set view matrix
        glm::vec3 cpos = m_camera.getPosition() + displacement; 
        glm::mat4 new_view = glm::lookAt(cpos, cpos + new_forward, glm::vec3{0,1,0});
        m_camera.setViewMatrix(new_view);
    }
    m_lastEventHandleTime = clock::now();
}

void Viewer::takeScreenshot()
{
    int padding = 5;
    sf::Texture screen;
    screen.create(m_window.getSize().x, m_window.getSize().y);
    screen.update(m_window);
    std::ostringstream filename_sstr;
    filename_sstr << screenshot_basename << std::setw(padding) << std::setfill('0') << m_screenshotCounter << ".png";
    if( !screen.copyToImage().saveToFile(filename_sstr.str()) )
    {
        LOG( error, "Error while saving the screenshot")
    }
    else
    {
        LOG( info, "Screenshot successfully taken : " << filename_sstr.str())
    }
    m_screenshotCounter++;
}

void Viewer::changeCameraMode()
{  
    Camera::CAMERA_BEHAVIOR mode = m_camera.getBehavior();
    m_camera.incrementBehavior();
}

bool Viewer::isRunning() const
{
    return m_applicationRunning;
}

void Viewer::display()
{
    m_window.display();
}

void Viewer::addShaderProgram( const ShaderProgramPtr & program )
{
    m_programs.insert( program );
}

void Viewer::reloadShaderPrograms()
{
    for( const ShaderProgramPtr & program : m_programs )
        program->reload();
}

Camera& Viewer::getCamera()
{
    return m_camera;
}

glm::vec3 Viewer::windowToWorld( const glm::vec3& windowCoordinate )
{
    sf::Vector2u size = m_window.getSize();
    return glm::unProject( windowCoordinate, m_camera.viewMatrix(), m_camera.projectionMatrix(), glm::vec4(0,0,size.x, size.y));
}

glm::vec3 Viewer::worldToWindow( const glm::vec3& worldCoordinate )
{
    sf::Vector2u size = m_window.getSize();
    return glm::project( worldCoordinate, m_camera.viewMatrix(), m_camera.projectionMatrix(), glm::vec4(0,0,size.x, size.y));
}

void Viewer::setBackgroundColor(const glm::vec4 & color)
{
    m_background_color = color;
    glcheck(glClearColor(m_background_color.r,
                         m_background_color.g,
                         m_background_color.b,
                         m_background_color.a));
}

const glm::vec4 & Viewer::getBackgroundColor() const
{
    return m_background_color;
}

//void Viewer::displayText(std::string text, Viewer::Duration duration)
//{
    //m_modeInformationText = text;
    //m_modeInformationTextDisappearanceTime = clock::now() + duration;
//}

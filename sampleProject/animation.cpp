#include <math.h>

#include <Viewer.hpp>
#include <glm/glm.hpp>

#include <ShaderProgram.hpp>

#include <lighting/PointLightRenderable.hpp>
#include <lighting/SpotLightRenderable.hpp>
#include <lighting/Material.hpp>

# include "./../include/ShaderProgram.hpp"
# include "./../include/FrameRenderable.hpp"

#include <dynamics/DynamicSystemRenderable.hpp>
#include <dynamics/ConstantForceField.hpp>
#include <dynamics/EulerExplicitSolver.hpp>

#include <dynamics/ParticleRenderable.hpp>

#include <texturing/CubeMapRenderable.hpp>
#include <texturing/TexturedLightedMeshRenderable.hpp>

#include "animation.hpp"

ShaderProgramPtr loadShader(Viewer& viewer, std::string vShader, std::string fShader){
    // Compile and link the shaders into a shader program
    ShaderProgramPtr shader = std::make_shared<ShaderProgram>(vShader, fShader);
	// Add the shader to the Viewer
	viewer.addShaderProgram(shader);
    return shader;
}

glm::mat4 getKartTranslation(glm::vec3 position){
    return getTranslationMatrix(position);
}

glm::mat4 getKartUpTranslation(){
    return getTranslationMatrix(glm::vec3(0., 0.2, 0.));
}

glm::mat4 getKartRotation(glm::vec3 lookAt, glm::vec3 normal){
    glm::vec3 normalizeNormal = glm::normalize(normal);
    glm::vec3 normalizeLookAt = glm::normalize(lookAt);
    glm::vec3 reference = glm::vec3(1.0, 0.0, 0.0);

    float normalDot = glm::dot(normalizeLookAt, reference);
    // Compute angle with the formula cos(theta) = dot(vec1, vec2) / (|vec1|*|vec2|)
    float angleACos = glm::acos(normalDot);

    // Solve the issu that angle after pi (pi < angle < 2*pi)
    // are egal to 2*pi - the true angle
    // we know when this occur when the cross product give a positive y.
    if(glm::cross(reference, normalizeLookAt)[1] > 0.0){
        angleACos = 2*M_PI-angleACos;
    }

    float angleY = glm::acos(glm::dot(glm::vec3(0.0, 1.0, 0.0), normalizeNormal));
    glm::vec3 axisY = glm::normalize(glm::cross(glm::vec3(0.0, 1.0, 0.0), normalizeNormal));
    
    // Set the rotation based on lookAt and normal:
    glm::mat4 kartRotation;

    kartRotation = getRotationMatrix(angleY, axisY);
    kartRotation *= getRotationMatrix(angleACos, glm::vec3(0.0, -1.0, 0.0));

    return kartRotation;
}

glm::mat4 getKartGlobalTransform(glm::vec3 position, glm::vec3 lookAt, glm::vec3 normal, float scale){
    glm::mat4 kartScale = getScaleMatrix(scale);
    glm::mat4 kartTranslation = getKartTranslation(position);
    glm::mat4 kartRotation = getKartRotation(lookAt, normal);
    glm::mat4 kartUpTranslation = getKartUpTranslation();

    return kartTranslation*kartRotation*kartUpTranslation*kartScale;
}

glm::mat4 getCameraInKartGlobalTransform(glm::vec3 position, glm::vec3 lookAt, glm::vec3 normal, float cameraUp){
    glm::mat4 kartTranslation = getKartTranslation(position);
    glm::mat4 kartRotation = getKartRotation(lookAt, normal);
    glm::mat4 kartUpTranslation = getKartUpTranslation();

    glm::mat4 cameraUpTranslation = getTranslationMatrix(glm::vec3(0., cameraUp, -0.2));
    glm::mat4 cameraRotation = getRotationMatrix(0.5*M_PI, glm::vec3(0., -1., 0.));

    return kartTranslation*kartRotation*kartUpTranslation*cameraRotation*cameraUpTranslation;
}

void collisions(Viewer& viewer, DynamicSystemPtr& system, DynamicSystemRenderablePtr &systemRenderable, ShaderProgramPtr shader){
    //Activate collision detection
    system->setCollisionsDetection(true);

    //Initialize the restitution coefficient for collision
    //1.0 = full elastic response
    //0.0 = full absorption
    system->setRestitution(0.99f);

    //Initialize a plane from 3 points and add it to the system as an obstacle
    glm::vec3 p1(-1.8,0.589,-2.0),p2(-0.8,0.589,-2.0), p3(-0.8,0.589,-4.0), p4(-1.8,0.589,-4.0);
    PlanePtr plane = std::make_shared<Plane>(p1, p2, p3);
    system->addPlaneObstacle(plane);

    glm::vec3 px,pv;
    float pm, pr;
    //Particle vs Plane collision
    {
        //Initialize a particle with position, velocity, mass and radius and add it to the system
        px = glm::vec3(-1.4,1.0,-3.0);
        pv = glm::vec3(0.0,0.0,0.0);
        pr = 0.1;
        pm = 1.0;
        ParticlePtr particle = std::make_shared<Particle>( px, pv, pm, pr);
        system->addParticle( particle );

        //Create a particleRenderable for each particle of the system
        ParticleRenderablePtr particleRenderable = std::make_shared<ParticleRenderable>(shader, particle);
        HierarchicalRenderable::addChild( systemRenderable, particleRenderable );
    }

    //Particle vs Particle collision
    {
        //Initialize two particles with position, velocity, mass and radius and add it to the system
        //One of the particle is fixed
        px = glm::vec3(-1.6,0.6,-3.0);
        pv = glm::vec3(0.0,0.0,0.0);
        pr = 0.1;
        pm = 1000.0;
        ParticlePtr particle1 = std::make_shared<Particle>( px, pv, pm, pr);
        particle1->setFixed(true);
        system->addParticle( particle1 );

        px = glm::vec3(-1.6,1.0,-3.0);
        pv = glm::vec3(0.0,-0.5,0.0);
        pr = 0.1;
        pm = 1.0;
        ParticlePtr particle2 = std::make_shared<Particle>( px, pv, pm, pr);
        system->addParticle( particle2 );

        //Create a particleRenderable for each particle of the system
        ParticleRenderablePtr particleRenderable1 = std::make_shared<ParticleRenderable>(shader, particle1);
        HierarchicalRenderable::addChild( systemRenderable, particleRenderable1 );
        ParticleRenderablePtr particleRenderable2 = std::make_shared<ParticleRenderable>(shader, particle2);
        HierarchicalRenderable::addChild( systemRenderable, particleRenderable2 );
    }

    //Initialize a force field that apply to all the particles of the system to simulate gravity
    //Add it to the system as a force field
    ConstantForceFieldPtr gravityForceField = std::make_shared<ConstantForceField>(system->getParticles(), DynamicSystem::gravity );
    system->addForceField( gravityForceField );
}


void initializeLights(Viewer& viewer, ShaderProgramPtr shader, int lightsNumber, glm::vec3* lightsPositions){
    //Define spot lights
    {
        glm::vec3 s_ambient(0.1,0.1,0.1), s_diffuse(0.3,0.3,0.3), s_specular(0.3,0.3,0.3);
        float s_constant=5.0, s_linear=1.0, s_quadratic=0.0;
        float s_innerCutOff = std::cos(glm::radians(20.0f));
        float s_outerCutOff = std::cos(glm::radians(40.0f));

        glm::mat4 spotLightLocalTransform;
        spotLightLocalTransform = getRotationMatrix(0.25*M_PI, glm::vec3(1., 0., 0.))*getScaleMatrix(0.03125);

        glm::vec3 spotLightPositions[8] = {
            glm::vec3( -0.366,    0.423,   -1.101),
            glm::vec3(  0.395,    0.423,   -1.101),
            glm::vec3( -0.366,    0.254,   -0.808),
            glm::vec3(  0.395,    0.254,   -0.808),
            glm::vec3( -0.366,    0.086,   -0.515),
            glm::vec3(  0.395,    0.086,   -0.515),
            glm::vec3(  0.395,    1.410,   -2.534),
            glm::vec3( -0.366,    1.410,   -2.534)
        };

        glm::vec3 s_direction = glm::normalize(glm::vec3(0.0,1.0,0.0));
        SpotLightPtr spotLight;
        SpotLightRenderablePtr spotLightRenderable;
        int idx = 0;
        for (; idx < 6; idx++){
            spotLight = std::make_shared<SpotLight>(spotLightPositions[idx], s_direction, 
                                                    s_ambient, s_diffuse, s_specular,
                                                    s_constant, s_linear, s_quadratic,
                                                    s_innerCutOff, s_outerCutOff);
            spotLightRenderable = std::make_shared<SpotLightRenderable>(shader, spotLight);
            spotLightRenderable->setLocalTransform(spotLightLocalTransform);
            viewer.addSpotLight(spotLight);
            viewer.addRenderable(spotLightRenderable);
        } 
        // idx = 6
        s_direction = glm::normalize(glm::vec3(0.0,-1.0,0.0));
        for (; idx < 8; idx++){
            spotLight = std::make_shared<SpotLight>(spotLightPositions[idx], s_direction, 
                                                    s_ambient, s_diffuse, s_specular,
                                                    s_constant, s_linear, s_quadratic,
                                                    s_innerCutOff, s_outerCutOff);
            spotLightRenderable = std::make_shared<SpotLightRenderable>(shader, spotLight);
            spotLightRenderable->setLocalTransform(spotLightLocalTransform);
            viewer.addSpotLight(spotLight);
            viewer.addRenderable(spotLightRenderable);
        } 
    }

    //Define point lights
    {
        glm::vec3 p_ambient(1.0,1.0,1.0), p_diffuse(1.0,1.0,1.0), p_specular(1.0,1.0,1.0);
        float p_constant=5.0, p_linear=1.0, p_quadratic=0.0;
        glm::mat4 localTransformation = getScaleMatrix(0.03125);

        PointLightPtr pointLight;
        for (int i = 0; i < lightsNumber; i++){
            pointLight = std::make_shared<PointLight>(lightsPositions[i], p_ambient, p_diffuse, p_specular, p_constant, p_linear, p_quadratic);

            PointLightRenderablePtr pointLightRenderable = std::make_shared<PointLightRenderable>(shader, pointLight);
            pointLightRenderable->setLocalTransform(localTransformation);
            viewer.addPointLight(pointLight);
            viewer.addRenderable(pointLightRenderable);
        }
    }
}


void initializeCubmap(Viewer& viewer){
    ShaderProgramPtr cubeMapShader = loadShader(viewer, "../../sfmlGraphicsPipeline/shaders/cubeMapVertex.glsl",
                                                        "../../sfmlGraphicsPipeline/shaders/cubeMapFragment.glsl");
    
    std::string cubemap_dir = "../../sfmlGraphicsPipeline/textures/skybox";
    auto cubemap = std::make_shared<CubeMapRenderable>(cubeMapShader, cubemap_dir);

    viewer.addRenderable(cubemap);
}


void initializeScene( Viewer& viewer ){
	
	ShaderProgramPtr defaultShader = loadShader(viewer, "./../../sfmlGraphicsPipeline/shaders/defaultVertex.glsl",
                                                        "./../../sfmlGraphicsPipeline/shaders/defaultFragment.glsl");
    
    ShaderProgramPtr textureShader = loadShader(viewer, "../../sfmlGraphicsPipeline/shaders/textureVertex.glsl",
                                                    "../../sfmlGraphicsPipeline/shaders/textureFragment.glsl");

    glm::vec3 ambiant(.5, .5, .5), diffuse(.3, .3, .3), specular(0., 0., 0.);
    float shininess = 0.3;
    MaterialPtr material = std::make_shared<Material>(ambiant, diffuse, specular, shininess);

    // Initialize the track and the ground
    const std::string trackMeshPath = "../../sfmlGraphicsPipeline/meshes/track.obj";
    const std::string trackTexturePath = "../../sfmlGraphicsPipeline/textures/trackTexture.png";
    auto track = std::make_shared<TexturedLightedMeshRenderable>(textureShader, trackMeshPath, material, trackTexturePath);

    const std::string standMeshPath = "../../sfmlGraphicsPipeline/meshes/stand.obj";
    const std::string standTexturePath = "../../sfmlGraphicsPipeline/textures/standTexture.png";
    auto stand = std::make_shared<TexturedLightedMeshRenderable>(textureShader, standMeshPath, material, standTexturePath);

    const std::string groundMeshPath = "../../sfmlGraphicsPipeline/meshes/ground.obj";
    const std::string groundTexturePath = "../../sfmlGraphicsPipeline/textures/groundTexture.png";
    auto ground = std::make_shared<TexturedLightedMeshRenderable>(textureShader, groundMeshPath, material, groundTexturePath);

    glm::mat4 globalTerrainTransform = getTranslationMatrix(0,0,0)*getScaleMatrix(3);
    track->setGlobalTransform(globalTerrainTransform);
    stand->setGlobalTransform(globalTerrainTransform);
    ground->setGlobalTransform(globalTerrainTransform);

    viewer.addRenderable(track);
    viewer.addRenderable(stand);
    viewer.addRenderable(ground);

    // Initialize the lights 
    int lightsNumber = NB_POSITION-1; // first and last positions are the same
    initializeLights(viewer, defaultShader, lightsNumber, POSITIONS);

    // Initialize the kart
    const std::string kartMeshPath = "../../sfmlGraphicsPipeline/meshes/kart.obj";
    const std::string kartTexturePath = "../../sfmlGraphicsPipeline/textures/kartTexture.png";
    auto kart = std::make_shared<TexturedMeshRenderable>(textureShader, kartMeshPath, kartTexturePath); 
     
    // Animate the kart on the track 
    glm::mat4 kartGlobalTransform, cameraGlobalTransform;
    float time = .0;

    int idx = 0;
    int part = 0;
    float cameraUp = .5;
    cameraGlobalTransform = getTranslationMatrix(glm::vec3(0.0, cameraUp, 0.0));
    viewer.getCamera().setViewMatrix(getTranslationMatrix(glm::vec3(0.0, -cameraUp, 0.0)));

    kartGlobalTransform = getKartGlobalTransform(POSITIONS[idx], LOOKS_AT[idx], NORMALS[idx], kartScale);
    kart->setGlobalTransform(kartGlobalTransform);

    viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time);

    // Camera out
    for( ; idx < partTimeStamp[part]; idx++){
        kartGlobalTransform = getKartGlobalTransform(POSITIONS[idx], LOOKS_AT[idx], NORMALS[idx], kartScale);
        kart->addGlobalTransformKeyframe(kartGlobalTransform, time);

        time += timeIncr[part];
    }

    viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time - timeIncr[part]);

    //Wait in place
    {
        time += waitTime - timeIncr[part];
        
        kart->addGlobalTransformKeyframe(kartGlobalTransform, time);

        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time - 0.5*waitTime);
        
        cameraGlobalTransform = getTranslationMatrix(glm::vec3(0.0, cameraUp, 0.0));
        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time - 0.5*waitTime);

        cameraGlobalTransform = getCameraInKartGlobalTransform(POSITIONS[idx-1], LOOKS_AT[idx-1], NORMALS[idx-1], cameraUp);

        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time - 0.3*waitTime);
        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time);
    }

    // Camera in
    part++;
    for ( ; part < NB_PART; part++){
        for( ; idx < partTimeStamp[part]; idx++){
            kartGlobalTransform = getKartGlobalTransform(POSITIONS[idx], LOOKS_AT[idx], NORMALS[idx], kartScale);
            cameraGlobalTransform = getCameraInKartGlobalTransform(POSITIONS[idx], LOOKS_AT[idx], NORMALS[idx], cameraUp);

            kart->addGlobalTransformKeyframe(kartGlobalTransform, time);
            viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time);

            time = time + timeIncr[part];            
        }
    }

    idx = 0;
    part = 0;
    for( ; idx < partTimeStamp[part]; idx++){
        kartGlobalTransform = getKartGlobalTransform(POSITIONS[idx], LOOKS_AT[idx], NORMALS[idx], kartScale);
        cameraGlobalTransform = getCameraInKartGlobalTransform(POSITIONS[idx], LOOKS_AT[idx], NORMALS[idx], cameraUp);

        kart->addGlobalTransformKeyframe(kartGlobalTransform, time);
        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time);

        time += timeIncr[part];
    }

    // Wait in place 
    {
        time += waitTime - timeIncr[part];
        cameraGlobalTransform = getCameraInKartGlobalTransform(POSITIONS[idx-1], LOOKS_AT[idx-1], NORMALS[idx-1], cameraUp);

        kart->addGlobalTransformKeyframe(kartGlobalTransform, time);
        
        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time - 0.8*waitTime);
        
        cameraGlobalTransform = getTranslationMatrix(glm::vec3(0.0, cameraUp, 0.));
        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time - 0.2*waitTime);
        viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time);
    }

    //camera out
    part++;
    for ( ; part < NB_PART; part++){
        for( ; idx < partTimeStamp[part]; idx++){
            kartGlobalTransform = getKartGlobalTransform(POSITIONS[idx], LOOKS_AT[idx], NORMALS[idx], kartScale);
            cameraGlobalTransform = getTranslationMatrix(glm::vec3(0.0, cameraUp, 0.));
            
            kart->addGlobalTransformKeyframe(kartGlobalTransform, time);

            time += timeIncr[part];
        }
    }
    time -= timeIncr[part-1];
    viewer.getCamera().addGlobalTransformKeyframe(cameraGlobalTransform, time);

    viewer.addRenderable(kart);
    
    //Add a physic animation
    //Initialize a dynamic system (Solver, Time step, Restitution coefficient)
    DynamicSystemPtr system = std::make_shared<DynamicSystem>();
    EulerExplicitSolverPtr solver = std::make_shared<EulerExplicitSolver>();
    system->setSolver(solver);
    system->setDt(0.01);

    DynamicSystemRenderablePtr systemRenderable = std::make_shared<DynamicSystemRenderable>(system);
    viewer.addRenderable(systemRenderable);

    collisions(viewer, system, systemRenderable, defaultShader);

    initializeCubmap(viewer);
}



int main(int argc, char* argv[]) 
{
	Viewer viewer(SCREEN_WIDTH, SCREEN_HEIGHT, BACKGROUND_COLOR);
	initializeScene(viewer);

	while( viewer.isRunning() )
	{
		viewer.handleEvent();
		viewer.animate();
		viewer.draw();
		viewer.display();
	}	

	return EXIT_SUCCESS;
}
#include "./../../include/dynamics/SpringForceField.hpp"

SpringForceField::SpringForceField(const ParticlePtr p1, const ParticlePtr p2, float stiffness, float equilibriumLength, float damping) :
    m_p1(p1),
    m_p2(p2),
    m_stiffness(stiffness),
    m_equilibriumLength(equilibriumLength),
    m_damping(damping)
{}

void SpringForceField::do_addForce()
{
    //TODO: Implement a damped spring
    //Functions to use:
    //glm::length( vec3 ): Return the length of a vector
    //glm::normalize( vec3 ): Return the normalization of a vector
    //Particle::getVelocity(), Particle::getForce(), Particle::setForce()
    //Nb:   Compute force ONLY IF the displacement length is above std::numeric_limits<float>::epsilon()
    //      Otherwise the computation is useless

    //Compute displacement vector
    glm::vec3 displacementVector = m_p1->getPosition() - m_p2->getPosition();

    //Compute displacement length
    float len = glm::length(displacementVector);

    //Compute spring force corresponding to the displacement 
    //If the displacement is measurable by the computer (otherwise no force)
    if ( std::numeric_limits<float>::epsilon() < abs(len)){
        float displacementLength =  len - m_equilibriumLength;
        glm::vec3 displacementDirection = glm::normalize(displacementVector);
        glm::vec3 f_k = - m_stiffness * displacementLength * displacementDirection;
        glm::vec3 f_kc = - m_damping * glm::dot((m_p1->getVelocity() - m_p2->getVelocity()), displacementDirection) * displacementDirection;

        glm::vec3 f_21 = f_k + f_kc;
        
        m_p1->setForce(m_p1->getForce() + f_21);
        m_p2->setForce(m_p2->getForce() - f_21);
    }
}

ParticlePtr SpringForceField::getParticle1() const
{
    return m_p1;
}

ParticlePtr SpringForceField::getParticle2() const
{
    return m_p2;
}

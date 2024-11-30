#include "./../../include/dynamics/EulerExplicitSolver.hpp"

EulerExplicitSolver::EulerExplicitSolver()
{

}

EulerExplicitSolver::~EulerExplicitSolver()
{

}

void EulerExplicitSolver::do_solve(const float& dt, std::vector<ParticlePtr>& particles)
{
    for(ParticlePtr p : particles)
    {
        if(!p->isFixed())
        {
            //Implement explicit euler solver
            //Functions to use:
            //Particle::getPosition(), Particle::getVelocity(), Particle::getMass(), Particle::getForce()
            //Particle::setPosition(), Particle::setVelocity()

            glm::vec3 newVelocity = p->getVelocity() + (dt / p->getMass()) * p->getForce();
            p->setVelocity( newVelocity );
            p->setPosition(p->getPosition() + dt * newVelocity);
        }
    }
}

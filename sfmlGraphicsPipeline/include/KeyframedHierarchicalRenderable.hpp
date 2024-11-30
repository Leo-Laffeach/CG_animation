#ifndef KEYFRAMED_HIERARCHICAL_RENDERABLE_HPP
#define KEYFRAMED_HIERARCHICAL_RENDERABLE_HPP

#include "HierarchicalRenderable.hpp"
#include "KeyframeCollection.hpp"

#include <glm/glm.hpp>

class GeometricTransformation;

class KeyframedHierarchicalRenderable : public HierarchicalRenderable
{
public:
    ~KeyframedHierarchicalRenderable();
    KeyframedHierarchicalRenderable( ShaderProgramPtr program);

    /**
     * \brief Add a keyframe for the local transformation of the renderable.
     *
     * Add a keyframe to m_localKeyframes described by a geometric transformation and a time.
     * \param transformation The geometric transformation of the keyframe.
     * \param time The time of the keyframe.
     */
    void addLocalTransformKeyframe( const GeometricTransformation& transformation, float time );

    /**
     * \brief Add a keyframe for the global transformation of the renderable.
     *
     * Add a keyframe to m_globalKeyframes described by a geometric transformation and a time.
     * \param transformation The geometric transformation of the keyframe.
     * \param time The time of the keyframe.
     */
    void addGlobalTransformKeyframe( const GeometricTransformation& transformation, float time );

protected:
    KeyframedHierarchicalRenderable():
        HierarchicalRenderable(nullptr)
    {}

    virtual void do_animate( float time );

private:
    KeyframeCollection m_localKeyframes; /*!< A collection of keyframes for the local transformation of renderable. */
    KeyframeCollection m_globalKeyframes; /*!< A collection of keyframes for the global transformation of renderable. */

};

typedef std::shared_ptr<KeyframedHierarchicalRenderable> KeyframedHierarchicalRenderablePtr;

#endif

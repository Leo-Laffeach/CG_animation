# include "./../include/KeyframeCollection.hpp"
# include <glm/gtx/compatibility.hpp>
#include <iostream>

void KeyframeCollection::add( const GeometricTransformation& transformation, float time )
{
    m_keyframes.insert( std::make_pair(time, transformation) );
}

glm::mat4 KeyframeCollection::interpolateTransformation( float time ) const
{
    //TODO: Complete the interpolation framework
    if( !m_keyframes.empty() )
    {
        //Handle the case where the time parameter is outside the keyframes time scope.
        std::map< float, GeometricTransformation >::const_iterator itFirstFrame = m_keyframes.begin();
        std::map< float, GeometricTransformation >::const_reverse_iterator itLastFrame = m_keyframes.rbegin();
        float effective_time = std::fmod(time, itLastFrame->first);

        //Get keyframes surrounding the time parameter
        std::array< Keyframe, 2 > result = getBoundingKeyframes( effective_time );

        //Compute the interpolating factor based on the time parameter and the surrounding keyframes times.
        Keyframe interpolatedKeyFrame;
        interpolatedKeyFrame.first = time;

        float interpolationFactor = (effective_time - result[0].first) / (result[1].first - result[0].first);


        //Interpolate each transformation component of the surrounding keyframes: orientation, translation, scale
        //  Use spherical linear interpolation for the orientation interpolation, glm::slerp(value1, value2, factor);
        //  Use linear interpolation for the translation and scale, glm::lerp(value1, value2, factor);

        interpolatedKeyFrame.second.setTranslation( glm::lerp(result[0].second.getTranslation(), result[1].second.getTranslation(), interpolationFactor) );
        interpolatedKeyFrame.second.setScale( glm::lerp(result[0].second.getScale(), result[1].second.getScale(), interpolationFactor) );
        interpolatedKeyFrame.second.setOrientation( glm::slerp(result[0].second.getOrientation(), result[1].second.getOrientation(), interpolationFactor) );

        //Build a matrix transformation from the orientation, translation and scale components
        return interpolatedKeyFrame.second.toMatrix();
    }
    else
    {
        return glm::mat4(1.0);
    }
}

bool KeyframeCollection::empty() const
{
    return m_keyframes.empty();
}

const std::map< float, GeometricTransformation > & KeyframeCollection::getKeyFrames() const
{
    return m_keyframes;
}

std::array< KeyframeCollection::Keyframe, 2 > KeyframeCollection::getBoundingKeyframes( float time ) const
{
    std::array< KeyframeCollection::Keyframe, 2 > result{ std::make_pair(0, GeometricTransformation()), std::make_pair(0, GeometricTransformation()) };
    std::map< float, GeometricTransformation >::const_iterator upper = m_keyframes.upper_bound(time);
    std::map< float, GeometricTransformation >::const_iterator lower = std::prev(upper);
    std::map< float, GeometricTransformation >::const_iterator end = m_keyframes.end();
    if(upper != end && lower != end )
    {
        result[0] = *lower;
        result[1] = *upper;
    }
    return result;
}

#ifndef UTILS_HPP
#define UTILS_HPP

/**@file
 * @brief Some useful functions to ease the development.
 */

#include "GeometricTransformation.hpp"
#include "Plane.hpp"

#include <array>
#include <vector>
#include <utility>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <Camera.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// float32 version
#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif

#ifndef M_TAU
#define M_TAU 6.283185307179586
#endif

// float32 version
#ifndef M_TAUf
#define M_TAUf 6.283185307179586f
#endif

/** @brief Get a random number uniformly sampled in [a,b[.
 *
 * This function returns a random number in [a,b[.
 * @return The random number generated. */
float random(float a, float b);

/** @brief Get a random valid color.
 *
 * This function creates for you a random color you can use to render your vertices.
 * @return The random color created. */
glm::vec4 randomColor();

/** @brief Get a color to represent a one dimensional parameter.
 *
 * This function allows to visually represent one dimensional parameter
 * thanks to a color code: cold colors (blue) for small values and hot colors
 * (red) for big values.
 *
 * @param factor The factor to visually represent.
 * @param low The minimum expected value for the factor (will return a blue color).
 * @param high The maximum expected value for the factor (will return a red color).
 * @return The generated color vector. */
glm::vec4 getColor( float factor, float low, float high );

void getUnitPlane(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> &tcoords);
void getIndexedUnitPlane(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> &tcoords, std::vector<glm::uvec3>& indices);
void getUnitCube(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> &tcoords);
void getUnitIndexedCube(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::uvec3>& indices);
void getUnitCylinder(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> & tcoords, unsigned int slices, bool vertex_normals=false);
void getUnitIndexedCylinder(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::uvec3> & indices, unsigned int slices);
void getUnitSphere(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> & tcoords, unsigned int strips, unsigned int slices, bool vertex_normals=false);
void getUnitIndexedSphere(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::uvec3> & indices, unsigned int strips, unsigned int slices);
void getUnitTorus(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> & tcoords, float tube, unsigned int strips, unsigned int slices, bool vertex_normals=false);
void getUnitIndexedTorus(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::uvec3> & indices, float tube, unsigned int strips, unsigned int slices);
void getUnitCone(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2> & tcoords, unsigned int slices, bool vertex_normals=false);
void getUnitIndexedCone(std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::uvec3> & indices, unsigned int slices);

glm::mat4 getTranslationMatrix(const glm::vec3 & tvec);
glm::mat4 getTranslationMatrix(float x, float y, float z);
glm::mat4 getTranslationMatrix(float x);
glm::mat4 getScaleMatrix(const glm::vec3 & svec);
glm::mat4 getScaleMatrix(float x, float y, float z);
glm::mat4 getScaleMatrix(float x);
glm::mat4 getRotationMatrix(float angle, const glm::vec3 & avec);
glm::mat4 getRotationMatrix(float angle, float x, float y, float z);

glm::mat4 lookAtModel(const glm::vec3 & position, const glm::vec3 & target, const glm::vec3 & forward);
glm::mat4 lookAtUp(const glm::vec3 & position, const glm::vec3 & target);
glm::mat4 lookAtUpModel(const glm::vec3 & position, const glm::vec3 & target);

template<typename T>
void unpack(const std::vector<glm::tvec4<T>> & packed, std::vector<T> & unpacked){
    unpacked.resize(packed.size() * 4, 0u);
    for (size_t i=0; i<packed.size(); ++i)
    {
        unpacked[4 * i + 0] = packed[i].x;
        unpacked[4 * i + 1] = packed[i].y;
        unpacked[4 * i + 2] = packed[i].z;
        unpacked[4 * i + 3] = packed[i].w;
    }
}

template<typename T>
void unpack(const std::vector<glm::tvec3<T>> & packed, std::vector<T> & unpacked){
    unpacked.resize(packed.size() * 3, 0u);
    for (size_t i=0; i<packed.size(); ++i)
    {
        unpacked[3 * i + 0] = packed[i].x;
        unpacked[3 * i + 1] = packed[i].y;
        unpacked[3 * i + 2] = packed[i].z;
    }
}

template<typename T>
void unpack(const std::vector<glm::tvec2<T>> & packed, std::vector<T> & unpacked){
    unpacked.resize(packed.size() * 2, 0u);
    for (size_t i=0; i<packed.size(); ++i)
    {
        unpacked[2 * i + 0] = packed[i].x;
        unpacked[2 * i + 1] = packed[i].y;
    }
}

template<typename T>
void pack(const std::vector<T> & unpacked, std::vector<glm::tvec4<T>> & packed){

    size_t packed_size = size_t(packed.size() / 4);
    packed.resize(packed_size);
    for (size_t i=0; i<packed.size(); ++i)
        packed[i] = glm::tvec4<T>(unpacked[4 * i + 0], unpacked[4 * i + 1], unpacked[4 * i + 2], unpacked[4 * i + 3]);
}

template<typename T>
void pack(const std::vector<T> & unpacked, std::vector<glm::tvec3<T>> & packed){
    size_t packed_size = size_t(packed.size() / 3);
    packed.resize(packed_size);
    for (size_t i=0; i<packed.size(); ++i)
        packed[i] = glm::tvec3<T>(unpacked[3 * i + 0], unpacked[3 * i + 1], unpacked[3 * i + 2]);
}

template<typename T>
void pack(const std::vector<T> & unpacked, std::vector<glm::tvec2<T>> & packed){
    size_t packed_size = size_t(packed.size() / 2);
    packed.resize(packed_size);
    for (size_t i=0; i<packed.size(); ++i)
        packed[i] = glm::tvec2<T>(unpacked[2 * i + 0], unpacked[2 * i + 1]);
}

// Utility function to deindex indexed data buffer

template<typename T>
void deindex(const std::vector<T> & indexed, const std::vector<glm::uvec3> & indices, std::vector<T> & deindexed){
    deindexed.resize(indices.size() * 3);
    for (size_t i=0; i<indices.size(); ++i)
    {
        deindexed[3 * i + 0] = indexed[indices[i].x];
        deindexed[3 * i + 1] = indexed[indices[i].y];
        deindexed[3 * i + 2] = indexed[indices[i].z];
    }
}

// Utility function to print a vector of any type supporting << operator

template<typename T>
static void print_vector(const std::string & name, const std::vector<T> & v){
    if (v.size() == 0){
        std::cout<<name<<" = []"<<std::endl;
    }
    std::cout<<name<<" = ["<<v[0];
    for (std::size_t i=1;i<v.size();++i){
        std::cout<<", \n"<<v[i];
    }
    std::cout<<"]"<<std::endl;
}


// Utility function for creating an std::vector with increasing values

template<typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}

#endif //UTILS

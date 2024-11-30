if( ${CMAKE_BINARY_DIR} STREQUAL ${CMAKE_SOURCE_DIR}/build )
else()
	message( FATAL_ERROR "Dear student, you just tried to launch"
	" cmake in another directory than build/. We encourage you to"
	" be more careful in the future and NEVER DO THAT AGAIN. Now "
	"the build system will stop, and you will remove " 
	"../CMakeCache.txt and ../CMakefiles/.\n--Your 3D Computer Graphics teachers.")
endif()

#==========================================
#Project options
#==========================================
cmake_minimum_required(VERSION 2.8)

#==========================================
#Project name
#==========================================
project(sfmlGraphicsPipeline-example)

#==========================================
#Building options
#==========================================
if (UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fopenmp")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
elseif(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -static-libgcc -static-libstdc++ -fopenmp")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -static-libgcc -static-libstdc++ -DDEBUG")
endif()
#==========================================
#Libraries path : glm, sfml, glew, opengl, freetype
#==========================================

#SFML GRAPHICS PIPELINE
message(${CMAKE_CURRENT_SOURCE_DIR})
set(SFML_GRAPHICS_PIPELINE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/include/" CACHE PATH "sfml graphics pipeline")
include_directories(${SFML_GRAPHICS_PIPELINE_INCLUDE_DIRS})
set(SFML_GRAPHICS_PIPELINE_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/build/libSFML_GRAPHICS_PIPELINE.a" CACHE FILEPATH "path to sfml graphics pipeline library")

#GLM Libraries
set(GLM_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/glm-0.9.7.1" CACHE PATH "glm")
include_directories(${GLM_INCLUDE_DIRS})

#GLEW Libraries
set(GLEW_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/glew-1.13.0/include" CACHE PATH "glew")
include_directories(${GLEW_INCLUDE_DIRS})
if(UNIX)
    set(GLEW_LIBRARIES "/usr/lib/x86_64-linux-gnu/libGLEW.so" CACHE FILEPATH "glew")
elseif(APPLE)
    message("APPLE platform not handled")
elseif(WIN32)
    set(GLEW_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/glew-1.13.0/lib/glew32.dll" CACHE FILEPATH "glew")
endif()

#SFML Libraries
if(UNIX)
    set(SFML_INCLUDE_DIRS "/usr/include/SFML" CACHE PATH "sfml")
    include_directories(${SFML_INCLUDE_DIRS})
    set(SFML_SYSTEM_LIBRARIES "/usr/lib/x86_64-linux-gnu/libsfml-system.so" CACHE FILEPATH "sfml")
    set(SFML_WINDOW_LIBRARIES "/usr/lib/x86_64-linux-gnu/libsfml-window.so" CACHE FILEPATH "sfml")
    set(SFML_GRAPHICS_LIBRARIES "/usr/lib/x86_64-linux-gnu/libsfml-graphics.so" CACHE FILEPATH "sfml")
elseif(APPLE)
    message("APPLE platform not handled")
elseif(WIN32)
    set(SFML_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/SFML-2.5.1/include" CACHE PATH "sfml")
    include_directories(${SFML_INCLUDE_DIRS})
    set(SFML_SYSTEM_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/SFML-2.5.1/build/lib/libsfml-system.a" CACHE FILEPATH "sfml")
    set(SFML_WINDOW_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/SFML-2.5.1/build/lib/libsfml-window.a" CACHE FILEPATH "sfml")
    set(SFML_GRAPHICS_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/SFML-2.5.1/build/lib/libsfml-graphics.a" CACHE FILEPATH "sfml")
    endif()

#FREETYPE Libraries	
set(FREETYPE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/freetype-2.5.5/include" CACHE PATH "freetype")
include_directories(${FREETYPE_INCLUDE_DIRS})
if(UNIX OR WIN32)
    set(FREETYPE_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/freetype-2.5.5/build/libfreetype.a" CACHE FILEPATH "freetype")
elseif(APPLE)
    message("APPLE platform not handled")
endif()	

#TINYOBJLOADER Libraries	
set(TINYOBJLOADER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/tinyobjloader/" CACHE PATH "tinyobjloader")
include_directories(${TINYOBJLOADER_INCLUDE_DIRS})	
if(UNIX OR WIN32)
    set(TINYOBJLOADER_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/tinyobjloader/build/libtinyobjloader.a" CACHE FILEPATH "tinyobjloader")
elseif(APPLE)
    message("APPLE platform not handled")
endif()


find_package(OpenGL REQUIRED)

#==============================================
#Project sources : src, exe
#==============================================
# Find all the cpp files in sampleProject
file(GLOB SOURCE_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" )

# For each of them define executable and link the libraries (sfmlGraphicsPipeline, OpenGL)
foreach( SOURCE_PATH IN LISTS SOURCE_PATHS)
    file(RELATIVE_PATH SOURCE_NAME ${CMAKE_CURRENT_SOURCE_DIR} ${SOURCE_PATH})
    get_filename_component(EXECUTABLE_NAME ${SOURCE_NAME} NAME_WE)
    add_executable(${EXECUTABLE_NAME} ${SOURCE_NAME})
    target_link_libraries(${EXECUTABLE_NAME} ${SFML_GRAPHICS_PIPELINE_LIBRARIES})
    target_link_libraries(${EXECUTABLE_NAME} ${SFML_SYSTEM_LIBRARIES})
    target_link_libraries(${EXECUTABLE_NAME} ${SFML_WINDOW_LIBRARIES})
    target_link_libraries(${EXECUTABLE_NAME} ${SFML_GRAPHICS_LIBRARIES})
    target_link_libraries(${EXECUTABLE_NAME} ${GLEW_LIBRARIES})
    target_link_libraries(${EXECUTABLE_NAME} ${FREETYPE_LIBRARIES})
    target_link_libraries(${EXECUTABLE_NAME} ${TINYOBJLOADER_LIBRARIES})
    if (OPENGL_FOUND)
        target_link_libraries(${EXECUTABLE_NAME} ${OPENGL_LIBRARIES})
        target_link_libraries(${EXECUTABLE_NAME} m)  # if you use maths.h
    endif()
endforeach()


# Windows executables struggles to the dlls so we copy them in the same directory
if(WIN32)
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/SFML-2.5.1/build/lib/sfml-system-2.dll" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/build")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/SFML-2.5.1/build/lib/sfml-window-2.dll" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/build")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/SFML-2.5.1/build/lib/sfml-graphics-2.dll" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/build")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../sfmlGraphicsPipeline/extlib/glew-1.13.0/lib/glew32.dll" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/build")
endif()

# Uncomment to compile in debug mode
# so you can have access to glcheck(...) macro
set(CMAKE_BUILD_TYPE "Debug")
message( "The build type is set to " ${CMAKE_BUILD_TYPE})

#ifndef CUBEMAPUTILS_HPP
#define CUBEMAPUTILS_HPP

#include <SFML/Graphics.hpp>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <array>
#include <string>

namespace cmutils{

    typedef std::array<sf::Image, 6> Cubemap;

    static const std::array<std::string, 6> face_names = {"right","left","top","bottom","front","back"};

    Cubemap load_cubemap(const std::string & cubemap_dir);

    void load_cubemap(const std::string & cubemap_dir, cmutils::Cubemap & cubemap);

    void save_cubemap(const Cubemap & cubemap, const std::string & cubemap_dir);

    unsigned int send_quad();

    unsigned int send_cubemap(const Cubemap & cubemap);

    void cubemap_to_panorama_draw(const Cubemap & cubemap, sf::RenderTexture & panorama_rt, unsigned int tBuffer, unsigned int cTid);

    void blur_panorama_draw(sf::RenderTexture & panorama_rt, unsigned int tBuffer, unsigned int csize, unsigned int ksize);

    void panorama_to_cubemap_draw(const sf::RenderTexture & panorama_rt, Cubemap & cubemap, unsigned int csize, unsigned int tBuffer);

    Cubemap blur_cubemap(const Cubemap & cubemap, unsigned int csize, unsigned int ksize);

    void blur_cubemap_directory(const std::string & skybox_dir, const std::string & blurred_skybox_dir, unsigned int csize, unsigned int ksize);
};
#endif
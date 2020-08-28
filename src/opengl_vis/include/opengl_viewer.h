//
// Created by kevin on 2020/4/25.
//

#ifndef CATKIN_WORKSPACE_OPENGL_VIEWER_H
#define CATKIN_WORKSPACE_OPENGL_VIEWER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb_image.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <shader.h>
#include <camera.h>
#include <config.h>

#include <string>
#include <iostream>
#include <vector>
#include <memory>

using namespace std;

namespace openglViewer{

class Texture{
public:
    typedef std::shared_ptr<Texture> Ptr;
    unsigned int id_;
    int width_, height_, nrChannels_;
    string fileDir_;

    unsigned char *data_;
private:
    static unsigned int factory_id_;

public:
    Texture();
    Texture(unsigned int id, string dir);
    ~Texture();

    static Texture::Ptr createTexture(string dir);

};


class OpenGLViewer{

public:
    // settings
    unsigned int window_width;
    unsigned int window_height;

    unsigned int point_num;
    unsigned int tri_num;
    unsigned int texture_num;

    float* vertices;
    unsigned int* indices;

    std::vector<Texture::Ptr> textures;

    string vsprofile, fsprofile;

public:
    OpenGLViewer();
    ~OpenGLViewer();

};

}

#endif
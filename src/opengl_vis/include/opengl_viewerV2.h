//
// Created by kevin on 2020/4/25.
//

#ifndef CATKIN_WORKSPACE_OPENGL_VIEWERV2_H
#define CATKIN_WORKSPACE_OPENGL_VIEWERV2_H

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

class TextureV2{
public:
    typedef std::shared_ptr<TextureV2> Ptr;
    unsigned int id_;
    int width_, height_, nrChannels_;
    string fileDir_;

    unsigned int point_num;
    unsigned int tri_num;

    float* vertices;
    unsigned int* indices;

    unsigned char *data_;
private:
    static unsigned int factory_id_;

public:
    TextureV2();
    TextureV2(unsigned int id, const char* info, int num);
    ~TextureV2();

    void normalize_texture();

    static TextureV2::Ptr createTexture(const char* info, int num);

};


class OpenGLViewerV2{

public:
    // settings
    unsigned int window_width;
    unsigned int window_height;

    unsigned int texture_num;

    std::vector<TextureV2::Ptr> textures;

    string vsprofile, fsprofile;

public:
    OpenGLViewerV2();
    ~OpenGLViewerV2();

    void pos_normalize();

};

}

#endif
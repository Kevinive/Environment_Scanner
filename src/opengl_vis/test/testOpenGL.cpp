//
// Created by kevin on 2020/4/23.
//

#include <iostream>
//# include <GL/glut.h>
//#include <stdio.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

int main(int argc, char** argv)
{
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

        GLFWwindow* window = glfwCreateWindow(800, 600, "LearnOpenGL", NULL, NULL);
        if (window == NULL)
        {
            std::cout << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return -1;
        }
        glfwMakeContextCurrent(window);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cout << "Failed to initialize GLAD" << std::endl;
            return -1;
        }

        glViewport(0, 0, 800, 600);



        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        while(!glfwWindowShouldClose(window))
        {
            glClear(GL_COLOR_BUFFER_BIT);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        glfwTerminate();

        return 0;


//    glutInit(&argc,argv);
//    //显示模式初始化
//    glutInitDisplayMode(GLUT_SINGLE|GLUT_RGB|GLUT_DEPTH);
//    //定义窗口大小
//    glutInitWindowSize(300,300);
//    //定义窗口位置
//    glutInitWindowPosition(100,100);
//    //创建窗口
//    glutCreateWindow("OpenGL Version");
//    const GLubyte* name = glGetString(GL_VENDOR); //返回负责当前OpenGL实现厂商的名字
//    const GLubyte* biaoshifu = glGetString(GL_RENDERER); //返回一个渲染器标识符，通常是个硬件平台
//    const GLubyte* OpenGLVersion =glGetString(GL_VERSION); //返回当前OpenGL实现的版本号
//    const GLubyte* gluVersion= gluGetString(GLU_VERSION); //返回当前GLU工具库版本
//    printf("OpenGL实现厂商的名字：%s\n", name);
//    printf("渲染器标识符：%s\n", biaoshifu);
//    printf("OOpenGL实现的版本号：%s\n",OpenGLVersion );
//    printf("OGLU工具库版本：%s\n", gluVersion);
//    return 0;
}
#include <glad/glad.h>
#include <GLFW/glfw3.h>
//#include "shader.h"
//#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
unsigned char* cvMat2TexInput(Mat& img);

int
main() {
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for Mac OSX
#endif

  VideoCapture cap(0);
  if(!cap.isOpened()) {
    // //std::cout << "Camera not opened!" << std::endl;
    return -1;
  }
  Mat frame;
  cap >> frame;
  int width = frame.cols;
  int height = frame.rows;
  unsigned char* image = cvMat2TexInput(frame);

  // glfw window creation
  GLFWwindow* window = glfwCreateWindow(width, height, "frame", NULL, NULL);
  if(window == NULL) {
    // std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  // glad: load all OpenGL function pointers
  if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    // std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  Shader ourShader("shader.vert", "shader.frag");
  float vertices[] = {
      //     Position       TexCoord
      -1.0f, 1.0f,  0.0f, 0.0f, 1.0f, // top left
      1.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top right
      -1.0f, -1.0f, 0.0f, 0.0f, 0.0f, // below left
      1.0f,  -1.0f, 0.0f, 1.0f, 0.0f  // below right
  };
  // Set up index
  unsigned int indices[] = {0, 1, 2, 1, 2, 3};

  unsigned int VAO, VBO, EBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, GL_STATIC_DRAW);
  glGenBuffers(1, &EBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glVertexAttribPointer(
      1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  unsigned int texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  while(!glfwWindowShouldClose(window)) {
    cap >> frame;
    image = cvMat2TexInput(frame);
    if(image) {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    } else {
      // std::cout << "Failed to load texture." << std::endl;
    }

    processInput(window);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    // Draw Rectangle
    ourShader.use();
    glBindTexture(GL_TEXTURE_2D, texture);
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwTerminate();
  return 0;
}

void
framebuffer_size_callback(GLFWwindow* window, int width, int height) {
  (void)window;
  glViewport(0, 0, width, height);
}

void
processInput(GLFWwindow* window) {
  if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
}

unsigned char*
cvMat2TexInput(Mat& img) {
  cvtColor(img, img, COLOR_BGR2RGB);
  flip(img, img, -1);
  return img.data;
}

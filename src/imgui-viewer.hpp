#ifndef IMGUI_VIEWER_HPP
#define IMGUI_VIEWER_HPP

#include <string>
#include <vector>

//#include "../imgui/libs/gl3w/GL/gl3w.hpp"
#include <SDL2/SDL.h>

class ImageTexture {
private:
  int width, height;
  GLuint my_opengl_texture;

public:
  ~ImageTexture();
  void setImage(cv::Mat* frame);       // from cv::Mat (BGR)
  void setImage(std::string filename); // from file
  void* getOpenglTexture();
  ImVec2 getSize();
};

ImageTexture::~ImageTexture() {
  glBindTexture(GL_TEXTURE_2D, 0); // unbind texture
  glDeleteTextures(1, &my_opengl_texture);
}

void
ImageTexture::setImage(cv::Mat* pframe) {
  width = pframe->cols;
  height = pframe->rows;

  glGenTextures(1, &my_opengl_texture);
  glBindTexture(GL_TEXTURE_2D, my_opengl_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

  // Some enviromnent doesn't support GP_BGR
  glTexImage2D(
      GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, (pframe->data));
}

void
ImageTexture::setImage(std::string filename) {
  cv::Mat frame = cv::imread(filename);
  setImage(&frame);
}

void*
ImageTexture::getOpenglTexture() {
  return (void*)(intptr_t)my_opengl_texture;
}

ImVec2
ImageTexture::getSize() {
  return ImVec2(width, height);
};

//---------------------------------------------------------------------

class ImageViewer {
private:
  // common
  SDL_Window* window;
  SDL_GLContext gl_context;
  ImGuiIO* io;

  // static contents
  ImVec4 clear_color;

  // dynamic contents
  std::vector<std::string> frame_names;
  std::vector<cv::Mat*> frames;
  float gain;

  void init();
  void initContents();
  void render();
  void showMainContents();

public:
  ImageViewer();
  bool handleEvent();
  void imshow(std::string, cv::Mat*);
  void show(cv::Mat*);
  void show();
  void exit();
  float getGain();
};

ImageViewer::ImageViewer() {
  init();
  initContents();

  io = &ImGui::GetIO();
  (void)&io;

  // dynamic contents
  gain = 1.0f;
}

#endif // defined IMGUI_VIEWER_HPP

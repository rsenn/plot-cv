#include <iostream>
#include <OpenGL/gl.h>
#include <GLFW/glfw3.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <SOIL.h>

using namespace std;

const char DEFAULT_WINDOW[] = "OpenCV's window";
const char ZOID_TEXTURE[] = "/full/path/to/the/image/zoid.jpg";

GLuint
create_zoid_texture(const char image_location[]) {
  GLuint zoid_texture = 0;
  glGenTextures(1, &zoid_texture);

  glBindTexture(GL_TEXTURE_2D, zoid_texture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  int t_width = 0, t_height = 0, t_channels = 0;
  unsigned char* zoidata_data = SOIL_load_image(image_location, &t_width, &t_height, &t_channels, 3);

  SOIL_create_OGL_texture(zoidata_data, t_width, t_height, t_channels, zoid_texture, 0);

  return zoid_texture;
}

void
draw_gl_texture(GLFWwindow* main_window) {
  glColor3f(1.0f, 1.0f, 1.0f);

  glBegin(GL_TRIANGLES);
  glTexCoord2f(0, 1);
  glVertex2f(-1, -1);

  glTexCoord2f(1, 1);
  glVertex2f(1, -1);

  glTexCoord2f(0, 0);
  glVertex2f(-1, 1);

  glTexCoord2f(1, 1);
  glVertex2f(1, -1);

  glTexCoord2f(1, 0);
  glVertex2f(1, 1);

  glTexCoord2f(0, 0);
  glVertex2f(-1, 1);
  glEnd();

  glfwSwapBuffers(main_window);
  glfwPollEvents();

  glFlush();
  glFinish();
}

cv::Mat
get_ocv_img_from_gl_img(GLuint ogl_texture_id) {
  glBindTexture(GL_TEXTURE_2D, ogl_texture_id);
  GLenum gl_texture_width, gl_texture_height;

  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, (GLint*)&gl_texture_width);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, (GLint*)&gl_texture_height);

  unsigned char* gl_texture_bytes = (unsigned char*)malloc(sizeof(unsigned char) * gl_texture_width * gl_texture_height * 3);
  glGetTexImage(GL_TEXTURE_2D, 0 /* mipmap level */, GL_BGR, GL_UNSIGNED_BYTE, gl_texture_bytes);

  return cv::Mat(gl_texture_height, gl_texture_width, CV_8UC3, gl_texture_bytes);
}

int
main() {
  glfwInit();
  GLFWwindow* main_window = glfwCreateWindow(640, 450, "OpenGL's window", nullptr, nullptr);
  glfwMakeContextCurrent(main_window);

  GLuint gl_texture = create_zoid_texture(ZOID_TEXTURE);
  glEnable(GL_TEXTURE_2D); // needed for intermediate mode!
  glBindTexture(GL_TEXTURE_2D, gl_texture);

  draw_gl_texture(main_window);

  // OCV part
  cvNamedWindow(DEFAULT_WINDOW);

  cv::Mat img = get_ocv_img_from_gl_img(gl_texture);

  cv::imshow(DEFAULT_WINDOW, img);
  cv::waitKey(0);
  return 0;
}

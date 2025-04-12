#define GLEW_STATIC
#include <stdlib.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GLFW/glfw3.h>

enum FBORenderTarget {
  NORMAL_FBO,
  NORMAL_TEXTURE,
  NORMAL_COLOR_RBO,
  NORMAL_DEPTH_RBO,
  MULTISAMPLING_FBO,
  MULTISAMPLING_TEXTURE,
  MULTISAMPLING_COLOR_RBO,
  MULTISAMPLING_DEPTH_RBO,
};
GLuint RenderRelatedIds[8];
int rw, rh; // <== This is the specified render canvas size, may not be the same as window size.
int
main() {
  /**
   * 1st we create the window and set it as the context.
   */
  if(glfwInit() == GLFW_FALSE)
    exit(EXIT_FAILURE);

  int width = 1024;
  int height = 768;

  GLFWwindow* window = glfwCreateWindow(width, height, "GLFW Window Example1", 0, 0);

  if(!window) {      // window wasn't created.
    glfwTerminate(); // free all resource from the glfwInit() call
    exit(EXIT_FAILURE);
  }
  glGenTextures(1, &RenderRelatedIds[NORMAL_TEXTURE]);
  glBindTexture(GL_TEXTURE_2D, RenderRelatedIds[NORMAL_TEXTURE]);
  {
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
  }
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, rw, rh, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
  glGenRenderbuffers(1, &RenderRelatedIds[NORMAL_COLOR_RBO]);
  glBindRenderbuffer(GL_RENDERBUFFER, RenderRelatedIds[NORMAL_COLOR_RBO]);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, rw, rh);
  glGenRenderbuffers(1, &RenderRelatedIds[NORMAL_DEPTH_RBO]);
  glBindRenderbuffer(GL_RENDERBUFFER, RenderRelatedIds[NORMAL_DEPTH_RBO]);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, rw, rh);
  glGenFramebuffers(1, &RenderRelatedIds[NORMAL_FBO]);
  glBindFramebuffer(GL_FRAMEBUFFER, RenderRelatedIds[NORMAL_FBO]);
  glFramebufferTexture2D(
      GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, RenderRelatedIds[NORMAL_TEXTURE], 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                            GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER,
                            RenderRelatedIds[NORMAL_COLOR_RBO]);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                            GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER,
                            RenderRelatedIds[NORMAL_DEPTH_RBO]);
  {
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER); // Check the FBO is ready.
    glViewport(0, 0, rw, rh);
    void* data = 0;
    glReadPixels(0, 0, rw, rh, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0,
               0,
               100,
               100); // width(), height() are functions returning the actual window size.
    glDeleteFramebuffers(1, &RenderRelatedIds[NORMAL_FBO]);
    glDeleteFramebuffers(1, &RenderRelatedIds[NORMAL_COLOR_RBO]);
    glDeleteFramebuffers(1, &RenderRelatedIds[NORMAL_DEPTH_RBO]);
    glDeleteFramebuffers(1, &RenderRelatedIds[NORMAL_TEXTURE]);

    glfwMakeContextCurrent(window); // set the window as the context.

    /**
     * At this point we have a window and a context is all setup!
     * 2nd. We jump into the main loop and dump stuff into the buffers to be rendered.
     *
     */

    // This function returns the value of the close flag of the specified window.
    while(glfwWindowShouldClose(window) == GLFW_FALSE) {
      glViewport(0, 0, width, height); // set the viewport of where we want to draw
      glClearColor(0, 0, 1, 1);        // clear color (R,G,B,A) for example blue
      glClear(GL_COLOR_BUFFER_BIT);    // clears the window to the color you want.

      // Normal drawing code would go here.

      glfwSwapBuffers(window); // swap buffer aka render.
      glfwPollEvents();        // listen for window events
    }

    /**
     * 3rd. release all resources and terminate glfw
     *
     */
    glfwDestroyWindow(window);
    glfwTerminate();
  }
}
#include <stdio.h>
#include <GL/gl.h>

int
main(int argc, char* argv[]) {
  printf("GL_MAJOR_VERSION = %d\n", GL_MAJOR_VERSION);
  printf("GL_MINOR_VERSION = %d\n", GL_MINOR_VERSION);
  printf("GL_VERSION = %d\n", GL_VERSION);

  return 0;
}

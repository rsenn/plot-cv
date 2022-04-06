#if _WIN32
#include <windows.h>
#else
#include <X11/Xlib.h>
#endif
#include <stdio.h>

//...

void
getScreenResolution(int* width, int* height) {
#if _WIN32
  width = (int)GetSystemMetrics(SM_CXSCREEN);
  height = (int)GetSystemMetrics(SM_CYSCREEN);
#else
  Display* disp = XOpenDisplay(NULL);
  Screen* scrn = DefaultScreenOfDisplay(disp);
  *width = scrn->width;
  *height = scrn->height;
#endif
}

int
main() {
  int width, height;
  getScreenResolution(&width, &height);
  printf("Screen resolution: %dx%d\n", width, height);
}

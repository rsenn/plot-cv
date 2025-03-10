#include <stdio.h>
#include <stddef.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/un.h>
#include <netpacket/packet.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <setjmp.h>
#include <glob.h>
#include <signal.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdint.h>
#include <limits.h>
#include <stdlib.h>
#include "imgui/imgui.h"

int
main() {
  printf("ImGuiWindowFlags_None = 0x%x\n", ImGuiWindowFlags_None);
  printf("ImGuiWindowFlags_NoTitleBar = 0x%x\n", ImGuiWindowFlags_NoTitleBar);
  printf("ImGuiWindowFlags_NoResize = 0x%x\n", ImGuiWindowFlags_NoResize);
  printf("ImGuiWindowFlags_NoMove = 0x%x\n", ImGuiWindowFlags_NoMove);
  printf("ImGuiWindowFlags_NoScrollbar = 0x%x\n", ImGuiWindowFlags_NoScrollbar);
  printf("ImGuiWindowFlags_NoScrollWithMouse = 0x%x\n", ImGuiWindowFlags_NoScrollWithMouse);
  printf("ImGuiWindowFlags_NoCollapse = 0x%x\n", ImGuiWindowFlags_NoCollapse);
  printf("ImGuiWindowFlags_AlwaysAutoResize = 0x%x\n", ImGuiWindowFlags_AlwaysAutoResize);
  printf("ImGuiWindowFlags_NoBackground = 0x%x\n", ImGuiWindowFlags_NoBackground);
  printf("ImGuiWindowFlags_NoSavedSettings = 0x%x\n", ImGuiWindowFlags_NoSavedSettings);
  printf("ImGuiWindowFlags_NoMouseInputs = 0x%x\n", ImGuiWindowFlags_NoMouseInputs);
  printf("ImGuiWindowFlags_MenuBar = 0x%x\n", ImGuiWindowFlags_MenuBar);
  printf("ImGuiWindowFlags_HorizontalScrollbar = 0x%x\n", ImGuiWindowFlags_HorizontalScrollbar);
  printf("ImGuiWindowFlags_NoFocusOnAppearing = 0x%x\n", ImGuiWindowFlags_NoFocusOnAppearing);
  printf("ImGuiWindowFlags_NoBringToFrontOnFocus = 0x%x\n",
         ImGuiWindowFlags_NoBringToFrontOnFocus);
  printf("ImGuiWindowFlags_AlwaysVerticalScrollbar = 0x%x\n",
         ImGuiWindowFlags_AlwaysVerticalScrollbar);
  printf("ImGuiWindowFlags_AlwaysHorizontalScrollbar = 0x%x\n",
         ImGuiWindowFlags_AlwaysHorizontalScrollbar);
  printf("ImGuiWindowFlags_AlwaysUseWindowPadding = 0x%x\n",
         ImGuiWindowFlags_AlwaysUseWindowPadding);
  printf("ImGuiWindowFlags_NoNavInputs = 0x%x\n", ImGuiWindowFlags_NoNavInputs);
  printf("ImGuiWindowFlags_NoNavFocus = 0x%x\n", ImGuiWindowFlags_NoNavFocus);
  printf("ImGuiWindowFlags_UnsavedDocument = 0x%x\n", ImGuiWindowFlags_UnsavedDocument);
  printf("ImGuiWindowFlags_NoNav = 0x%x\n", ImGuiWindowFlags_NoNav);
  printf("ImGuiWindowFlags_NoDecoration = 0x%x\n", ImGuiWindowFlags_NoDecoration);
  printf("ImGuiWindowFlags_NoInputs = 0x%x\n", ImGuiWindowFlags_NoInputs);
  printf("ImGuiWindowFlags_NavFlattened = 0x%x\n", ImGuiWindowFlags_NavFlattened);
  printf("ImGuiWindowFlags_ChildWindow = 0x%x\n", ImGuiWindowFlags_ChildWindow);
  printf("ImGuiWindowFlags_Tooltip = 0x%x\n", ImGuiWindowFlags_Tooltip);
  printf("ImGuiWindowFlags_Popup = 0x%x\n", ImGuiWindowFlags_Popup);
  printf("ImGuiWindowFlags_Modal = 0x%x\n", ImGuiWindowFlags_Modal);
  printf("ImGuiWindowFlags_ChildMenu = 0x%x\n", ImGuiWindowFlags_ChildMenu);
}

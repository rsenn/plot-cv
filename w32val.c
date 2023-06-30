#include <windows.h>
#include <stdio.h>
#include <math.h>

int
main() {
  printf("GENERIC_READ = %i\n", GENERIC_READ);
  printf("FILE_SHARE_READ = %i\n", FILE_SHARE_READ);
  printf("FILE_SHARE_WRITE = %i\n", FILE_SHARE_WRITE);
  printf("FILE_SHARE_DELETE = %i\n", FILE_SHARE_DELETE);
  printf("OPEN_EXISTING = %i\n", OPEN_EXISTING);
  printf("FILE_ATTRIBUTE_NORMAL = %i\n", FILE_ATTRIBUTE_NORMAL);
  printf("INVALID_HANDLE_VALUE = %i\n", INVALID_HANDLE_VALUE);
  printf("PAGE_WRITECOPY = %i\n", PAGE_WRITECOPY);
  printf("NULL = %i\n", NULL);
  printf("FILE_MAP_COPY = %i\n", FILE_MAP_COPY);
  printf("#define FOREGROUND_BLUE (1 << %.0f)\n", log2(FOREGROUND_BLUE));
  printf("#define FOREGROUND_GREEN (1 << %.0f)\n", log2(FOREGROUND_GREEN));
  printf("#define FOREGROUND_RED (1 << %.0f)\n", log2(FOREGROUND_RED));
  printf("#define FOREGROUND_INTENSITY (1 << %.0f)\n", log2(FOREGROUND_INTENSITY));
  printf("#define BACKGROUND_BLUE (1 << %.0f)\n", log2(BACKGROUND_BLUE));
  printf("#define BACKGROUND_GREEN (1 << %.0f)\n", log2(BACKGROUND_GREEN));
  printf("#define BACKGROUND_RED (1 << %.0f)\n", log2(BACKGROUND_RED));
  printf("#define BACKGROUND_INTENSITY (1 << %.0f)\n", log2(BACKGROUND_INTENSITY));
  printf("#define COMMON_LVB_LEADING_BYTE (1 << %.0f)\n", log2(COMMON_LVB_LEADING_BYTE));
  printf("#define COMMON_LVB_TRAILING_BYTE (1 << %.0f)\n", log2(COMMON_LVB_TRAILING_BYTE));
  printf("#define COMMON_LVB_GRID_HORIZONTAL (1 << %.0f)\n", log2(COMMON_LVB_GRID_HORIZONTAL));
  printf("#define COMMON_LVB_GRID_LVERTICAL (1 << %.0f)\n", log2(COMMON_LVB_GRID_LVERTICAL));
  printf("#define COMMON_LVB_GRID_RVERTICAL (1 << %.0f)\n", log2(COMMON_LVB_GRID_RVERTICAL));
  printf("#define COMMON_LVB_REVERSE_VIDEO (1 << %.0f)\n", log2(COMMON_LVB_REVERSE_VIDEO));
  printf("#define COMMON_LVB_UNDERSCORE (1 << %.0f)\n", log2(COMMON_LVB_UNDERSCORE));
  return 0;
}
printf("FOREGROUND_BLUE = %d\n", FOREGROUND_BLUE);
printf("FOREGROUND_GREEN = %d\n", FOREGROUND_GREEN);
printf("FOREGROUND_RED = %d\n", FOREGROUND_RED);
printf("FOREGROUND_INTENSITY = %d\n", FOREGROUND_INTENSITY);
printf("BACKGROUND_BLUE = %d\n", BACKGROUND_BLUE);
printf("BACKGROUND_GREEN = %d\n", BACKGROUND_GREEN);
printf("BACKGROUND_RED = %d\n", BACKGROUND_RED);
printf("BACKGROUND_INTENSITY = %d\n", BACKGROUND_INTENSITY);
printf("COMMON_LVB_LEADING_BYTE = %d\n", COMMON_LVB_LEADING_BYTE);
printf("COMMON_LVB_TRAILING_BYTE = %d\n", COMMON_LVB_TRAILING_BYTE);
printf("COMMON_LVB_GRID_HORIZONTAL = %d\n", COMMON_LVB_GRID_HORIZONTAL);
printf("COMMON_LVB_GRID_LVERTICAL = %d\n", COMMON_LVB_GRID_LVERTICAL);
printf("COMMON_LVB_GRID_RVERTICAL = %d\n", COMMON_LVB_GRID_RVERTICAL);
printf("COMMON_LVB_REVERSE_VIDEO = %d\n", COMMON_LVB_REVERSE_VIDEO);
printf("COMMON_LVB_UNDERSCORE = %d\n", COMMON_LVB_UNDERSCORE);

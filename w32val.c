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
  printf("FILE_ATTRIBUTE_READONLY = %d\n", FILE_ATTRIBUTE_READONLY);
  printf("FILE_ATTRIBUTE_HIDDEN = %d\n", FILE_ATTRIBUTE_HIDDEN);
  printf("FILE_ATTRIBUTE_SYSTEM = %d\n", FILE_ATTRIBUTE_SYSTEM);
  printf("FILE_ATTRIBUTE_DIRECTORY = %d\n", FILE_ATTRIBUTE_DIRECTORY);
  printf("FILE_ATTRIBUTE_ARCHIVE = %d\n", FILE_ATTRIBUTE_ARCHIVE);
  printf("FILE_ATTRIBUTE_DEVICE = %d\n", FILE_ATTRIBUTE_DEVICE);
  printf("FILE_ATTRIBUTE_NORMAL = %d\n", FILE_ATTRIBUTE_NORMAL);
  printf("FILE_ATTRIBUTE_TEMPORARY = %d\n", FILE_ATTRIBUTE_TEMPORARY);
  printf("FILE_ATTRIBUTE_SPARSE_FILE = %d\n", FILE_ATTRIBUTE_SPARSE_FILE);
  printf("FILE_ATTRIBUTE_REPARSE_POINT = %d\n", FILE_ATTRIBUTE_REPARSE_POINT);
  printf("FILE_ATTRIBUTE_COMPRESSED = %d\n", FILE_ATTRIBUTE_COMPRESSED);
  printf("FILE_ATTRIBUTE_OFFLINE = %d\n", FILE_ATTRIBUTE_OFFLINE);
  printf("FILE_ATTRIBUTE_NOT_CONTENT_INDEXED = %d\n", FILE_ATTRIBUTE_NOT_CONTENT_INDEXED);
  printf("FILE_ATTRIBUTE_ENCRYPTED = %d\n", FILE_ATTRIBUTE_ENCRYPTED);
  printf("sizeof(WCHAR) = %d\n", sizeof(WCHAR));
  printf("sizeof(TCHAR) = %d\n", sizeof(TCHAR));
  printf("sizeof(wchar_t) = %d\n", sizeof(wchar_t));
  return 0;
}

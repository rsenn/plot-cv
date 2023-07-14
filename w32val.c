#include <windows.h>
#include <stdio.h>

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
  printf("sizeof(WCHAR) = %zu\n", sizeof(WCHAR));
  printf("sizeof(TCHAR) = %zu\n", sizeof(TCHAR));
  printf("sizeof(wchar_t) = %zu\n", sizeof(wchar_t));
  return 0;
}

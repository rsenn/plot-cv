#define _GNU_SOURCE 1
#include <stdio.h>
#include <stddef.h>
#include <termios.h>

int
main() {
  printf("sizeof(struct termios) = %zu\n", sizeof(struct termios));

  printf("offsetof(struct termios, c_iflag) = %zu\n", offsetof(struct termios, c_iflag));
  printf("sizeof(((struct termios*)0)->c_iflag) = %zu\n", sizeof(((struct termios*)0)->c_iflag));
  printf("offsetof(struct termios, c_oflag) = %zu\n", offsetof(struct termios, c_oflag));
  printf("sizeof(((struct termios*)0)->c_oflag) = %zu\n", sizeof(((struct termios*)0)->c_oflag));
  printf("offsetof(struct termios, c_cflag) = %zu\n", offsetof(struct termios, c_cflag));
  printf("sizeof(((struct termios*)0)->c_cflag) = %zu\n", sizeof(((struct termios*)0)->c_cflag));
  printf("offsetof(struct termios, c_lflag) = %zu\n", offsetof(struct termios, c_lflag));
  printf("sizeof(((struct termios*)0)->c_lflag) = %zu\n", sizeof(((struct termios*)0)->c_lflag));
  printf("offsetof(struct termios, c_line) = %zu\n", offsetof(struct termios, c_line));
  printf("sizeof(((struct termios*)0)->c_line) = %zu\n", sizeof(((struct termios*)0)->c_line));
  printf("offsetof(struct termios, c_cc) = %zu\n", offsetof(struct termios, c_cc));
  printf("sizeof(((struct termios*)0)->c_cc) = %zu\n", sizeof(((struct termios*)0)->c_cc));
  return 0;
}

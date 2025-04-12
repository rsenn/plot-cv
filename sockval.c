#include <stdio.h>
#include <stddef.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <termio.h>
#include <fcntl.h>
#include <math.h>
#include <float.h>

//#ifndef OFFSETOF
#define OFFSETOF(type, field) ((size_t) & ((type*)0)->field)
//#endif

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte) \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'), (byte & 0x20 ? '1' : '0'), \
      (byte & 0x10 ? '1' : '0'), (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), \
      (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')
enum test { A, B, C, D };

typedef int dim2[10][20];
int
main() {
  struct sockaddr_in sa;
  struct timeval tv;
  struct termio tio;
  struct termios tios;
  struct pollfd pfd;
  typedef int dim1[10];

  dim2 test;

  printf("sizeof(enum test) = %zx\n", sizeof(enum test));
  printf("FLT_RADIX = %zx\n", FLT_RADIX);

  printf("%d %lu\n", FD_SETSIZE, sizeof(struct timeval));
  printf("sa.sin_family %lu %lu\n",
         OFFSETOF(struct sockaddr_in, sin_port),
         sizeof(sa.sin_family));
  printf("sa.sin_port %lu %lu\n", OFFSETOF(struct sockaddr_in, sin_port), sizeof(sa.sin_port));
  printf("sa.sin_addr %lu %lu\n", OFFSETOF(struct sockaddr_in, sin_addr), sizeof(sa.sin_addr));
  printf("sizeof(sa) %lu\n", sizeof(struct sockaddr_in));
  printf("sizeof(struct termio) %lu\n", sizeof(struct termio));
  printf("sizeof(tio.c_iflag) %lu\n", sizeof(tio.c_iflag));
  printf("OFFSETOF(struct termio,c_line) %lu\n", OFFSETOF(struct termio, c_line));
  printf("sizeof(tio.c_line) %lu\n", sizeof(tio.c_line));
  printf("OFFSETOF(struct termio,c_cc) %lu\n", OFFSETOF(struct termio, c_cc));
  printf("sizeof(tio.c_cc) %lu\n", sizeof(tio.c_cc));

  printf("sizeof(struct termios) %lu\n", sizeof(struct termios));
  printf("sizeof(tios.c_iflag) %lu\n", sizeof(tios.c_iflag));
  printf("OFFSETOF(struct termios,c_line) %lu\n", OFFSETOF(struct termios, c_line));
  printf("sizeof(tios.c_line) %lu\n", sizeof(tios.c_line));
  printf("OFFSETOF(struct termios,c_cc) %lu\n", OFFSETOF(struct termios, c_cc));
  printf("sizeof(tios.c_cc) %lu\n", sizeof(tios.c_cc));
  /*printf("OFFSETOF(struct termios,c_ispeed) %lu\n", OFFSETOF(struct termios,c_ispeed));
  printf("sizeof(tios.c_ispeed) %lu\n", sizeof(tios.c_ispeed));
  printf("OFFSETOF(struct termios,c_ospeed) %lu\n", OFFSETOF(struct termios,c_ospeed));
  printf("sizeof(tios.c_ospeed) %lu\n", sizeof(tios.c_ospeed));*/
  /*int32_t     num = 0x80000000;
  printf("num = %s\n", "0x80000000");
  printf("num %i\n", num);
  printf("s|%s|'0x%08x'|\n", "0x80000000", (uint32_t)num);
  printf("-num 0x%08x\n", num);
  num = 0x80045430;
  printf("num = %s\n", "0x80045430");
  printf("num %i\n", num);
  printf("s|%s|'0x%08x'|\n", "0x80045430", (uint32_t)num);
  printf("-num 0x%08x\n", num);
  num = 0x80045432;
  printf("num = %s\n", "0x80045432");
  printf("num %i\n", num);
  printf("s|%s|'0x%08x'|\n", "0x80045432", (uint32_t)num);
  printf("-num 0x%08x\n", num);
  num = 0x80045438;
  printf("num = %s\n", "0x80045438");
  printf("num %i\n", num);
  printf("s|%s|'0x%08x'|\n", "0x80045438", (uint32_t)num);
  printf("-num 0x%08x\n", num);
  num = 0x80045439;
  printf("num = %s\n", "0x80045439");
  printf("num %i\n", num);
  printf("s|%s|'0x%08x'|\n", "0x80045439", (uint32_t)num);
  printf("-num 0x%08x\n", num);
  num = 0x80045440;
  printf("num = %s\n", "0x80045440");
  printf("num %i\n", num);
  printf("s|%s|'0x%08x'|\n", "0x80045440", (uint32_t)num);
  printf("-num 0x%08x\n", num);
*/

  printf("SOL_SOCKET %lu\n", SOL_SOCKET);
#ifdef O_ACCMODE
  printf("O_ACCMODE = 0x%x\n", O_ACCMODE);
#endif
#ifdef O_APPEND
  printf("O_APPEND = 0x%x\n", O_APPEND);
#endif
#ifdef O_ASYNC
  printf("O_ASYNC = 0x%x\n", O_ASYNC);
#endif
#ifdef O_BLKSEEK
  printf("O_BLKSEEK = 0x%x\n", O_BLKSEEK);
#endif
#ifdef O_CLOEXEC
  printf("O_CLOEXEC = 0x%x\n", O_CLOEXEC);
#endif
#ifdef O_CREAT
  printf("O_CREAT = 0x%x\n", O_CREAT);
#endif
#ifdef O_DIRECT
  printf("O_DIRECT = 0x%x\n", O_DIRECT);
#endif
#ifdef O_DIRECTORY
  printf("O_DIRECTORY = 0x%x\n", O_DIRECTORY);
#endif
#ifdef O_DSYNC
  printf("O_DSYNC = 0x%x\n", O_DSYNC);
#endif
#ifdef O_EXCL
  printf("O_EXCL = 0x%x\n", O_EXCL);
#endif
#ifdef O_INVISIBLE
  printf("O_INVISIBLE = 0x%x\n", O_INVISIBLE);
#endif
#ifdef O_LARGEFILE
  printf("O_LARGEFILE = 0x%x\n", O_LARGEFILE);
#endif
#ifdef O_NDELAY
  printf("O_NDELAY = 0x%x\n", O_NDELAY);
#endif
#ifdef O_NOATIME
  printf("O_NOATIME = 0x%x\n", O_NOATIME);
#endif
#ifdef O_NOCTTY
  printf("O_NOCTTY = 0x%x\n", O_NOCTTY);
#endif
#ifdef O_NOFOLLOW
  printf("O_NOFOLLOW = 0x%x\n", O_NOFOLLOW);
#endif
#ifdef O_NONBLOCK
  printf("O_NONBLOCK = 0x%x\n", O_NONBLOCK);
#endif
#ifdef O_PATH
  printf("O_PATH = 0x%x\n", O_PATH);
#endif
#ifdef O_RDONLY
  printf("O_RDONLY = 0x%x\n", O_RDONLY);
#endif
#ifdef O_RDWR
  printf("O_RDWR = 0x%x\n", O_RDWR);
#endif
#ifdef O_RSYNC
  printf("O_RSYNC = 0x%x\n", O_RSYNC);
#endif
#ifdef O_SYNC
  printf("O_SYNC = 0x%x\n", O_SYNC);
#endif
#ifdef O_TMPFILE
  printf("O_TMPFILE = 0x%x\n", O_TMPFILE);
#endif
#ifdef O_TRUNC
  printf("O_TRUNC = 0x%x\n", O_TRUNC);
#endif
#ifdef O_WRONLY
  printf("O_WRONLY = 0x%x\n", O_WRONLY);
#endif

  printf("sizeof(socklen_t) %lu\n", sizeof(socklen_t));
  printf("sizeof(struct timeval) %lu\n", sizeof(struct timeval));
  printf("sizeof(tv.tv_sec) %lu\n", sizeof(tv.tv_sec));
  printf("sizeof(tv.tv_usec) %lu\n", sizeof(tv.tv_usec));
  printf("sizeof(float) %lu\n", sizeof(float));
  printf("sizeof(double) %lu\n", sizeof(double));
  printf("sizeof(long double) %lu\n", sizeof(long double));
  printf("sizeof(int) %lu\n", sizeof(int));
  printf("sizeof(long) %lu\n", sizeof(long));
  printf("sizeof(long long) %lu\n", sizeof(long long));
  float nan_f = NAN;
  double nan_d = NAN;
  printf("(uint32_t)NAN = %08x\n", *(uint32_t*)&nan_f);
  printf("(uint64_t)NAN = %016llx\n", *(uint64_t*)&nan_d);
  enum { A = 1, B = 2, C = 3 } e;
  printf("sizeof(e) %lu\n", sizeof(e));
  printf("sizeof(off_t) %lu\n", sizeof(off_t));
  printf("POLLIN  0b" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(POLLIN));
  printf("POLLPRI 0b" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(POLLPRI));
  printf("POLLOUT 0b" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(POLLOUT));
  printf("POLLERR 0b" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(POLLERR));
  printf("POLLHUP 0b" BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(POLLHUP));
  printf("#define POLLIN\t%d\n", POLLIN);
  printf("#define POLLOUT\t%d\n", POLLOUT);
  printf("#define POLLERR\t%d\n", POLLERR);
  printf("#define POLLHUP\t%d\n", POLLHUP);
}

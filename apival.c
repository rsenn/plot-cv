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

static inline int
escape_char_pred(int c) {
  switch(c) {
    case 8: return 'b';
    case 9: return 't';
    case 10: return 'n';
    case 11: return 'v';
    case 12: return 'f';
    case 13: return 'r';
    case 39: return '\'';
    case 92: return '\\';
  }
  if(c < 0x20 || c == 127)
    return 'x';

  return 0;
}

typedef struct numbers_s {
  union {
    uint32_t fl_i;
    float fl;
  };
  short sh;
  union {
    uint64_t db_i;
    double db;
  };
} numbers_t;

int
main() {
  struct timespec ts;
  numbers_t n = {};
  jmp_buf jmpb;
  glob_t gl;
  struct termios tio;
  n.fl_i = 0x40490fdb;
  n.db_i = 0x4005bf0a8b145769;
  printf("n.fl = %f\n", n.fl);
  printf("n.db = %lf\n", n.db);

  printf("ts.tv_sec %lu %lu\n", offsetof(struct timespec, tv_sec), sizeof(ts.tv_sec));
  printf("ts.tv_nsec %lu %lu\n", offsetof(struct timespec, tv_nsec), sizeof(ts.tv_nsec));
  printf("sizeof(struct timespec) %lu\n", sizeof(struct timespec));
  printf("sizeof(clockid_t) %lu\n", sizeof(clockid_t));
  printf("const PAGE_SIZE = %d;\n", sysconf(_SC_PAGE_SIZE));
  printf("const PAGE_SIZE = 0x%x;\n", sysconf(_SC_PAGE_SIZE));
  printf("const CLOCK_REALTIME = %d;\n", CLOCK_REALTIME);
  printf("const CLOCK_REALTIME_COARSE = %d;\n", CLOCK_REALTIME_COARSE);
  printf("const CLOCK_MONOTONIC = %d;\n", CLOCK_MONOTONIC);
  printf("const CLOCK_MONOTONIC_COARSE = %d;\n", CLOCK_MONOTONIC_COARSE);
  printf("const CLOCK_MONOTONIC_RAW = %d;\n", CLOCK_MONOTONIC_RAW);
  printf("const CLOCK_BOOTTIME = %d;\n", CLOCK_BOOTTIME);
  printf("const CLOCK_PROCESS_CPUTIME_ID = %d;\n", CLOCK_PROCESS_CPUTIME_ID);
  printf("const CLOCK_THREAD_CPUTIME_ID = %d;\n", CLOCK_THREAD_CPUTIME_ID);
  printf("sizeof(long long) = %zu\n", sizeof(long long));
  printf("sizeof(struct termios) = %zu\n", sizeof(struct termios));
  printf("%-10s = %d\n", "SIGHUP", SIGHUP);
  printf("%-10s = %d\n", "SIGINT", SIGINT);
  printf("%-10s = %d\n", "SIGQUIT", SIGQUIT);
  printf("%-10s = %d\n", "SIGILL", SIGILL);
  printf("%-10s = %d\n", "SIGTRAP", SIGTRAP);
  printf("%-10s = %d\n", "SIGABRT", SIGABRT);
  printf("%-10s = %d\n", "SIGBUS", SIGBUS);
  printf("%-10s = %d\n", "SIGFPE", SIGFPE);
  printf("%-10s = %d\n", "SIGKILL", SIGKILL);
  printf("%-10s = %d\n", "SIGUSR1", SIGUSR1);
  printf("%-10s = %d\n", "SIGSEGV", SIGSEGV);
  printf("%-10s = %d\n", "SIGUSR2", SIGUSR2);
  printf("%-10s = %d\n", "SIGPIPE", SIGPIPE);
  printf("%-10s = %d\n", "SIGALRM", SIGALRM);
  printf("%-10s = %d\n", "SIGTERM", SIGTERM);
  printf("%-10s = %d\n", "SIGSTKFLT", SIGSTKFLT);
  printf("%-10s = %d\n", "SIGCHLD", SIGCHLD);
  printf("%-10s = %d\n", "SIGCONT", SIGCONT);
  printf("%-10s = %d\n", "SIGSTOP", SIGSTOP);
  printf("%-10s = %d\n", "SIGTSTP", SIGTSTP);
  printf("%-10s = %d\n", "SIGTTIN", SIGTTIN);
  printf("%-10s = %d\n", "SIGTTOU", SIGTTOU);
  printf("%-10s = %d\n", "SIGURG", SIGURG);
  printf("%-10s = %d\n", "SIGXCPU", SIGXCPU);
  printf("%-10s = %d\n", "SIGXFSZ", SIGXFSZ);
  printf("%-10s = %d\n", "SIGVTALRM", SIGVTALRM);
  printf("%-10s = %d\n", "SIGPROF", SIGPROF);
  printf("%-10s = %d\n", "SIGWINCH", SIGWINCH);
  printf("%-10s = %d\n", "SIGIO", SIGIO);
  printf("%-10s = %d\n", "SIGPWR", SIGPWR);
  printf("%-10s = %d\n", "SIGSYS", SIGSYS);
  printf("INT32_MIN = %" PRIi32 "\n", INT32_MIN);
  printf("INT32_MAX = %" PRIi32 "\n", INT32_MAX);
  printf("INT64_MIN = %" PRIi64 "\n", INT64_MIN);
  printf("INT64_MAX = %" PRIi64 "\n", INT64_MAX);
  struct sockaddr_in si;
  struct sockaddr_in6 si6;
  struct sockaddr_ll sl;
  struct sockaddr_storage ss;
  struct sockaddr_un su;

  printf("sizeof(struct sockaddr_in) = %zu\n", sizeof(struct sockaddr_in));
  printf("sizeof(struct sockaddr_in6) = %zu\n", sizeof(struct sockaddr_in6));
  // printf("sizeof(struct sockaddr_in_pad) = %zu\n", sizeof(struct sockaddr_in_pad));
  printf("sizeof(struct sockaddr_ll) = %zu\n", sizeof(struct sockaddr_ll));
  // printf("sizeof(struct sockaddr_nl) = %zu\n", sizeof(struct sockaddr_nl));
  // printf("sizeof(struct sockaddr_pkt) = %zu\n", sizeof(struct sockaddr_pkt));
  printf("sizeof(struct sockaddr_storage) = %zu\n", sizeof(struct sockaddr_storage));
  printf("sizeof(struct sockaddr_un) = %zu\n", sizeof(struct sockaddr_un));
  char map[256];

  for(size_t i = 0; i < 256; i++) map[i] = escape_char_pred(i);
  for(size_t i = 0; i < 256; i++) { printf("%s0x%02x", i > 0 ? ", " : "", map[i]); }
  printf("PATH_MAX = %zu\n", PATH_MAX);
}

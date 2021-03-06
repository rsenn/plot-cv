#include <stdio.h>
#include <stddef.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/mman.h>
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
}

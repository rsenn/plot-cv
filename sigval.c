#include <stdio.h>
#include <signal.h>
#include <stddef.h>

int
main() {
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
  typedef struct sigaction sa_t;
  struct sigaction* sa = 0;
  printf("sizeof(struct sigaction) = %zu\n", sizeof(struct sigaction));
  printf("offsetof(struct sigaction, sa_handler) = %zu\n",
         offsetof(struct sigaction, sa_handler));
  printf("sizeof(sa->sa_handler) = %zu\n", sizeof(sa->sa_handler));
  printf("offsetof(struct sigaction, sa_mask) = %zu\n", offsetof(struct sigaction, sa_mask));
  printf("sizeof(sa->sa_mask) = %zu\n", sizeof(sa->sa_mask));
  printf("offsetof(struct sigaction, sa_flags) = %zu\n", offsetof(struct sigaction, sa_flags));
  printf("sizeof(sa->sa_flags) = %zu\n", sizeof(sa->sa_flags));
  printf("sizeof(sigset_t) = %zu\n", sizeof(sigset_t));
  printf("NSIG = %zu\n", NSIG);
  printf("SIGRTMAX = %zu\n", SIGRTMAX);
  printf("SIGRTMIN = %zu\n", SIGRTMIN);
}

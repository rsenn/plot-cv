#include <stdio.h>
#include <sys/wait.h>
#include <signal.h>

int
main() {
  printf("#define %-9s  0x%x\n", "WNOWAIT", WNOWAIT);
  printf("#define %-9s %2i\n", "WUNTRACED", WUNTRACED);
  printf("#define %-9s %2i\n", "WNOHANG", WNOHANG);
  printf("#define %-9s %2i\n", "SIGHUP", SIGHUP);
  printf("#define %-9s %2i\n", "SIGINT", SIGINT);
  printf("#define %-9s %2i\n", "SIGQUIT", SIGQUIT);
  printf("#define %-9s %2i\n", "SIGILL", SIGILL);
  printf("#define %-9s %2i\n", "SIGTRAP", SIGTRAP);
  printf("#define %-9s %2i\n", "SIGABRT", SIGABRT);
  printf("#define %-9s %2i\n", "SIGBUS", SIGBUS);
  printf("#define %-9s %2i\n", "SIGFPE", SIGFPE);
  printf("#define %-9s %2i\n", "SIGKILL", SIGKILL);
  printf("#define %-9s %2i\n", "SIGUSR1", SIGUSR1);
  printf("#define %-9s %2i\n", "SIGSEGV", SIGSEGV);
  printf("#define %-9s %2i\n", "SIGUSR2", SIGUSR2);
  printf("#define %-9s %2i\n", "SIGPIPE", SIGPIPE);
  printf("#define %-9s %2i\n", "SIGALRM", SIGALRM);
  printf("#define %-9s %2i\n", "SIGTERM", SIGTERM);
  printf("#define %-9s %2i\n", "SIGSTKFLT", SIGSTKFLT);
  printf("#define %-9s %2i\n", "SIGCHLD", SIGCHLD);
  printf("#define %-9s %2i\n", "SIGCONT", SIGCONT);
  printf("#define %-9s %2i\n", "SIGSTOP", SIGSTOP);
  printf("#define %-9s %2i\n", "SIGTSTP", SIGTSTP);
  printf("#define %-9s %2i\n", "SIGTTIN", SIGTTIN);
  printf("#define %-9s %2i\n", "SIGTTOU", SIGTTOU);
  printf("#define %-9s %2i\n", "SIGURG", SIGURG);
  printf("#define %-9s %2i\n", "SIGXCPU", SIGXCPU);
  printf("#define %-9s %2i\n", "SIGXFSZ", SIGXFSZ);
  printf("#define %-9s %2i\n", "SIGVTALRM", SIGVTALRM);
  printf("#define %-9s %2i\n", "SIGPROF", SIGPROF);
  printf("#define %-9s %2i\n", "SIGWINCH", SIGWINCH);
  printf("#define %-9s %2i\n", "SIGIO", SIGIO);
  printf("#define %-9s %2i\n", "SIGPWR", SIGPWR);
  printf("#define %-9s %2i\n", "SIGSYS", SIGSYS);

  return 0;
}
 

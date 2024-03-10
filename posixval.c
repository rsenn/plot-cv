#include <aio.h>
#include <arpa/inet.h>
#include <assert.h>
#include <complex.h>
#include <cpio.h>
#include <ctype.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <fenv.h>
#include <float.h>
#include <fmtmsg.h>
#include <fnmatch.h>
#include <ftw.h>
#include <glob.h>
#include <grp.h>
#include <iconv.h>
#include <inttypes.h>
#include <iso646.h>
#include <langinfo.h>
#include <libgen.h>
#include <limits.h>
#include <locale.h>
#include <math.h>
#include <monetary.h>
#include <mqueue.h>
#include <ndbm.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <nl_types.h>
#include <poll.h>
#include <pthread.h>
#include <pwd.h>
#include <regex.h>
#include <sched.h>
#include <search.h>
#include <semaphore.h>
#include <setjmp.h>
#include <signal.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/resource.h>
#include <sys/select.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/un.h>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <syslog.h>
#include <tar.h>
#include <termios.h>
#include <tgmath.h>
#include <time.h>
#include <ulimit.h>
#include <unistd.h>
#include <utime.h>
#include <utmpx.h>
#include <wchar.h>
#include <wctype.h>
#include <wordexp.h>

int
main(int argc, char* argv[]) {
  printf("S_IFMT = 0x%04x\n", S_IFMT);
  printf("S_IFSOCK = 0x%04x\n", S_IFSOCK);
  printf("S_IFLNK = 0x%04x\n", S_IFLNK);
  printf("S_IFREG = 0x%04x\n", S_IFREG);
  printf("S_IFBLK = 0x%04x\n", S_IFBLK);
  printf("S_IFDIR = 0x%04x\n", S_IFDIR);
  printf("S_IFCHR = 0x%04x\n", S_IFCHR);
  printf("S_IFIFO = 0x%04x\n", S_IFIFO);

  printf("c_iflag IGNBRK = %i\n", IGNBRK);
  printf("c_iflag BRKINT = %i\n", BRKINT);
  printf("c_iflag IGNPAR = %i\n", IGNPAR);
  printf("c_iflag PARMRK = %i\n", PARMRK);
  printf("c_iflag INPCK = %i\n", INPCK);
  printf("c_iflag ISTRIP = %i\n", ISTRIP);
  printf("c_iflag INLCR = %i\n", INLCR);
  printf("c_iflag IGNCR = %i\n", IGNCR);
  printf("c_iflag ICRNL = %i\n", ICRNL);
  printf("c_iflag IUCLC = %i\n", IUCLC);
  printf("c_iflag IXON = %i\n", IXON);
  printf("c_iflag IXANY = %i\n", IXANY);
  printf("c_iflag IXOFF = %i\n", IXOFF);
  printf("c_iflag IMAXBEL = %i\n", IMAXBEL);
  printf("c_iflag IUTF8 = %i\n", IUTF8);
  printf("c_oflag OPOST = %i\n", OPOST);
  printf("c_oflag OLCUC = %i\n", OLCUC);
  printf("c_oflag ONLCR = %i\n", ONLCR);
  printf("c_oflag OCRNL = %i\n", OCRNL);
  printf("c_oflag ONOCR = %i\n", ONOCR);
  printf("c_oflag ONLRET = %i\n", ONLRET);
  printf("c_oflag OFILL = %i\n", OFILL);
  printf("c_oflag OFDEL = %i\n", OFDEL);
  printf("c_oflag NLDLY = %i\n", NLDLY);
  printf("c_oflag NL0 = %i\n", NL0);
  printf("c_oflag NL1 = %i\n", NL1);
  printf("c_oflag CRDLY = %i\n", CRDLY);
  printf("c_oflag CR0 = %i\n", CR0);
  printf("c_oflag CR1 = %i\n", CR1);
  printf("c_oflag CR2 = %i\n", CR2);
  printf("c_oflag CR3 = %i\n", CR3);
  printf("c_oflag TABDLY = %i\n", TABDLY);
  printf("c_oflag TAB0 = %i\n", TAB0);
  printf("c_oflag TAB1 = %i\n", TAB1);
  printf("c_oflag TAB2 = %i\n", TAB2);
  printf("c_oflag TAB3 = %i\n", TAB3);
  printf("c_oflag XTABS = %i\n", XTABS);
  printf("c_oflag BSDLY = %i\n", BSDLY);
  printf("c_oflag BS0 = %i\n", BS0);
  printf("c_oflag BS1 = %i\n", BS1);
  printf("c_oflag VTDLY = %i\n", VTDLY);
  printf("c_oflag VT0 = %i\n", VT0);
  printf("c_oflag VT1 = %i\n", VT1);
  printf("c_oflag FFDLY = %i\n", FFDLY);
  printf("c_oflag FF0 = %i\n", FF0);
  printf("c_oflag FF1 = %i\n", FF1);
  printf("c_cflag CBAUD = %i\n", CBAUD);
  printf("c_cflag B0 = %i\n", B0);
  printf("c_cflag B50 = %i\n", B50);
  printf("c_cflag B75 = %i\n", B75);
  printf("c_cflag B110 = %i\n", B110);
  printf("c_cflag B134 = %i\n", B134);
  printf("c_cflag B150 = %i\n", B150);
  printf("c_cflag B200 = %i\n", B200);
  printf("c_cflag B300 = %i\n", B300);
  printf("c_cflag B600 = %i\n", B600);
  printf("c_cflag B1200 = %i\n", B1200);
  printf("c_cflag B1800 = %i\n", B1800);
  printf("c_cflag B2400 = %i\n", B2400);
  printf("c_cflag B4800 = %i\n", B4800);
  printf("c_cflag B9600 = %i\n", B9600);
  printf("c_cflag B19200 = %i\n", B19200);
  printf("c_cflag B38400 = %i\n", B38400);
  printf("c_cflag EXTA = %i\n", EXTA);
  printf("c_cflag EXTB = %i\n", EXTB);
  printf("c_cflag CSIZE = %i\n", CSIZE);
  printf("c_cflag CS5 = %i\n", CS5);
  printf("c_cflag CS6 = %i\n", CS6);
  printf("c_cflag CS7 = %i\n", CS7);
  printf("c_cflag CS8 = %i\n", CS8);
  printf("c_cflag CSTOPB = %i\n", CSTOPB);
  printf("c_cflag CREAD = %i\n", CREAD);
  printf("c_cflag PARENB = %i\n", PARENB);
  printf("c_cflag PARODD = %i\n", PARODD);
  printf("c_cflag HUPCL = %i\n", HUPCL);
  printf("c_cflag CLOCAL = %i\n", CLOCAL);
  printf("c_cflag CBAUDEX = %i\n", CBAUDEX);
  printf("c_cflag B57600 = %i\n", B57600);
  printf("c_cflag B115200 = %i\n", B115200);
  printf("c_cflag B230400 = %i\n", B230400);
  printf("c_cflag B460800 = %i\n", B460800);
  printf("c_cflag B500000 = %i\n", B500000);
  printf("c_cflag B576000 = %i\n", B576000);
  printf("c_cflag B921600 = %i\n", B921600);
  printf("c_cflag B1000000 = %i\n", B1000000);
  printf("c_cflag B1152000 = %i\n", B1152000);
  printf("c_cflag B1500000 = %i\n", B1500000);
  printf("c_cflag B2000000 = %i\n", B2000000);
  printf("c_cflag B2500000 = %i\n", B2500000);
  printf("c_cflag B3000000 = %i\n", B3000000);
  printf("c_cflag B3500000 = %i\n", B3500000);
  printf("c_cflag B4000000 = %i\n", B4000000);
  printf("c_cflag CIBAUD = %i\n", CIBAUD);
  printf("c_cflag CMSPAR = %i\n", CMSPAR);
  printf("c_cflag CRTSCTS = %i\n", CRTSCTS);
  printf("c_lflag ISIG = %i\n", ISIG);
  printf("c_lflag ICANON = %i\n", ICANON);
  printf("c_lflag XCASE = %i\n", XCASE);
  printf("c_lflag ECHO = %i\n", ECHO);
  printf("c_lflag ECHOE = %i\n", ECHOE);
  printf("c_lflag ECHOK = %i\n", ECHOK);
  printf("c_lflag ECHONL = %i\n", ECHONL);
  printf("c_lflag NOFLSH = %i\n", NOFLSH);
  printf("c_lflag ECHOCTL = %i\n", ECHOCTL);
  printf("c_lflag ECHOPRT = %i\n", ECHOPRT);
  printf("c_lflag ECHOKE = %i\n", ECHOKE);

  return 0;
}

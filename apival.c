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
#include <libwebsockets.h>
#include <errno.h>
#include <sys/ptrace.h>
#include <sys/inotify.h>
#include <portmidi.h>
#include <linux/random.h>
#include <zlib.h>
#include "quickjs/qjs-net/libwebsockets/include/libwebsockets.h"

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
  //  __int128 i128;
  struct timespec ts;
  numbers_t n = {};
  jmp_buf jmpb;
  glob_t gl;
  struct termios tio;
  n.fl_i = 0x40490fdb;
  n.db_i = 0x4005bf0a8b145769;
  // printf("n.fl = %f\n", n.fl);
  // printf("n.db = %lf\n", n.db);

  // printf("ts.tv_sec %lu %lu\n", offsetof(struct timespec, tv_sec), sizeof(ts.tv_sec));
  // printf("ts.tv_nsec %lu %lu\n", offsetof(struct timespec, tv_nsec), sizeof(ts.tv_nsec));
  // printf("sizeof(struct timespec) %lu\n", sizeof(struct timespec));
  // printf("sizeof(clockid_t) %lu\n", sizeof(clockid_t));
  // printf("const PAGE_SIZE = %ld;\n", sysconf(_SC_PAGE_SIZE));
  // printf("const PAGE_SIZE = 0x%lx;\n", sysconf(_SC_PAGE_SIZE));
  // printf("const CLOCK_REALTIME = %d;\n", CLOCK_REALTIME);
  // printf("const CLOCK_REALTIME_COARSE = %d;\n", CLOCK_REALTIME_COARSE);
  // printf("const CLOCK_MONOTONIC = %d;\n", CLOCK_MONOTONIC);
  // printf("const CLOCK_MONOTONIC_COARSE = %d;\n", CLOCK_MONOTONIC_COARSE);
  // printf("const CLOCK_MONOTONIC_RAW = %d;\n", CLOCK_MONOTONIC_RAW);
  // printf("const CLOCK_BOOTTIME = %d;\n", CLOCK_BOOTTIME);
  // printf("const CLOCK_PROCESS_CPUTIME_ID = %d;\n", CLOCK_PROCESS_CPUTIME_ID);
  // printf("const CLOCK_THREAD_CPUTIME_ID = %d;\n", CLOCK_THREAD_CPUTIME_ID);
  // printf("sizeof(long long) = %zu\n", sizeof(long long));
  // printf("sizeof(struct termios) = %zu\n", sizeof(struct termios));
  // printf("%-10s = %d\n", "SIGHUP", SIGHUP);
  // printf("%-10s = %d\n", "SIGINT", SIGINT);
  // printf("%-10s = %d\n", "SIGQUIT", SIGQUIT);
  // printf("%-10s = %d\n", "SIGILL", SIGILL);
  // printf("%-10s = %d\n", "SIGTRAP", SIGTRAP);
  // printf("%-10s = %d\n", "SIGABRT", SIGABRT);
  // printf("%-10s = %d\n", "SIGBUS", SIGBUS);
  // printf("%-10s = %d\n", "SIGFPE", SIGFPE);
  // printf("%-10s = %d\n", "SIGKILL", SIGKILL);
  // printf("%-10s = %d\n", "SIGUSR1", SIGUSR1);
  // printf("%-10s = %d\n", "SIGSEGV", SIGSEGV);
  // printf("%-10s = %d\n", "SIGUSR2", SIGUSR2);
  // printf("%-10s = %d\n", "SIGPIPE", SIGPIPE);
  // printf("%-10s = %d\n", "SIGALRM", SIGALRM);
  // printf("%-10s = %d\n", "SIGTERM", SIGTERM);
  // printf("%-10s = %d\n", "SIGSTKFLT", SIGSTKFLT);
  // printf("%-10s = %d\n", "SIGCHLD", SIGCHLD);
  // printf("%-10s = %d\n", "SIGCONT", SIGCONT);
  // printf("%-10s = %d\n", "SIGSTOP", SIGSTOP);
  // printf("%-10s = %d\n", "SIGTSTP", SIGTSTP);
  // printf("%-10s = %d\n", "SIGTTIN", SIGTTIN);
  // printf("%-10s = %d\n", "SIGTTOU", SIGTTOU);
  // printf("%-10s = %d\n", "SIGURG", SIGURG);
  // printf("%-10s = %d\n", "SIGXCPU", SIGXCPU);
  // printf("%-10s = %d\n", "SIGXFSZ", SIGXFSZ);
  // printf("%-10s = %d\n", "SIGVTALRM", SIGVTALRM);
  // printf("%-10s = %d\n", "SIGPROF", SIGPROF);
  // printf("%-10s = %d\n", "SIGWINCH", SIGWINCH);
  // printf("%-10s = %d\n", "SIGIO", SIGIO);
  // printf("%-10s = %d\n", "SIGPWR", SIGPWR);
  // printf("%-10s = %d\n", "SIGSYS", SIGSYS);
  // printf("INT32_MIN = %" PRIi32 "\n", INT32_MIN);
  // printf("INT32_MAX = %" PRIi32 "\n", INT32_MAX);
  // printf("INT64_MIN = %" PRIi64 "\n", INT64_MIN);
  // printf("INT64_MAX = %" PRIi64 "\n", INT64_MAX);
  struct sockaddr_in si;
  struct sockaddr_in6 si6;
  struct sockaddr_ll sl;
  struct sockaddr_storage ss;
  struct sockaddr_un su;

  // printf("sizeof(struct sockaddr_in) = %zu\n", sizeof(struct sockaddr_in));
  // printf("sizeof(struct sockaddr_in6) = %zu\n", sizeof(struct sockaddr_in6));
  // printf("sizeof(struct sockaddr_in_pad) = %zu\n", sizeof(struct sockaddr_in_pad));
  // printf("sizeof(struct sockaddr_ll) = %zu\n", sizeof(struct sockaddr_ll));
  // printf("sizeof(struct sockaddr_nl) = %zu\n", sizeof(struct sockaddr_nl));
  // printf("sizeof(struct sockaddr_pkt) = %zu\n", sizeof(struct sockaddr_pkt));
  // printf("sizeof(struct sockaddr_storage) = %zu\n", sizeof(struct sockaddr_storage));
  // printf("sizeof(struct sockaddr_un) = %zu\n", sizeof(struct sockaddr_un));
  char map[256];

  for(size_t i = 0; i < 256; i++) map[i] = escape_char_pred(i);
  for(size_t i = 0; i < 256; i++) { printf("%s0x%02x", i > 0 ? ", " : "", map[i]); }
  // printf("PATH_MAX = %i\n", PATH_MAX);
  // printf("LWS_PRE = %i\n", LWS_PRE);
  // printf("NSIG = %i\n", NSIG);
  // printf("SIGRTMAX = %i\n", SIGRTMAX);
  // printf("SIGRTMIN = %i\n", SIGRTMIN);
  // printf("sizeof(dev_t) = %zu\n", sizeof(dev_t));
  // printf("sizeof(mode_t) = %zu\n", sizeof(mode_t));
  // printf("sizeof(socklen_t) = %zu\n", sizeof(socklen_t));
  // printf("sizeof(sa_family_t) = %zu\n", sizeof(sa_family_t));
  // printf("sizeof(struct timeval) = %zu\n", sizeof(struct timeval));
  struct timeval tv;
  // printf("sizeof(tv.tv_sec) = %zu\n", sizeof(tv.tv_sec));
  // printf("sizeof(struct timespec) = %zu\n", sizeof(struct timespec));
  // printf("SOCK_STREAM = %i\n", SOCK_STREAM);
  // printf("SOCK_DGRAM = %i\n", SOCK_DGRAM);
  printf("SHUT_RD = %i\n", SHUT_RD);
  printf("SHUT_WR = %i\n", SHUT_WR);
  printf("SHUT_RDWR = %i\n", SHUT_RDWR);
  printf("SO_ERROR = %i\n", SO_ERROR);
  printf("SO_DEBUG = %i\n", SO_DEBUG);
  printf("SO_REUSEADDR = %i\n", SO_REUSEADDR);
  printf("SO_KEEPALIVE = %i\n", SO_KEEPALIVE);
  printf("SO_DONTROUTE = %i\n", SO_DONTROUTE);
  printf("SO_BROADCAST = %i\n", SO_BROADCAST);
  printf("SO_OOBINLINE = %i\n", SO_OOBINLINE);
  printf("SO_REUSEPORT = %i\n", SO_REUSEPORT);
  printf("SO_SNDBUF = %i\n", SO_SNDBUF);
  printf("SO_RCVBUF = %i\n", SO_RCVBUF);
  printf("SO_NO_CHECK = %i\n", SO_NO_CHECK);
  printf("SO_PRIORITY = %i\n", SO_PRIORITY);
  printf("SO_BSDCOMPAT = %i\n", SO_BSDCOMPAT);
  printf("SO_PASSCRED = %i\n", SO_PASSCRED);
  printf("SO_PEERCRED = %i\n", SO_PEERCRED);
  printf("SO_SECURITY_AUTHENTICATION = %i\n", SO_SECURITY_AUTHENTICATION);
  printf("SO_SECURITY_ENCRYPTION_TRANSPORT = %i\n", SO_SECURITY_ENCRYPTION_TRANSPORT);
  printf("SO_SECURITY_ENCRYPTION_NETWORK = %i\n", SO_SECURITY_ENCRYPTION_NETWORK);
  printf("SO_BINDTODEVICE = %i\n", SO_BINDTODEVICE);
  printf("SO_ATTACH_FILTER = %i\n", SO_ATTACH_FILTER);
  printf("SO_DETACH_FILTER = %i\n", SO_DETACH_FILTER);
  printf("SO_GET_FILTER = %i\n", SO_GET_FILTER);
  printf("SO_PEERNAME = %i\n", SO_PEERNAME);
  printf("SO_TIMESTAMP = %i\n", SO_TIMESTAMP);
  printf("SO_PEERSEC = %i\n", SO_PEERSEC);
  printf("SO_PASSSEC = %i\n", SO_PASSSEC);
  printf("SO_TIMESTAMPNS = %i\n", SO_TIMESTAMPNS);
  printf("SO_MARK = %i\n", SO_MARK);
  printf("SO_TIMESTAMPING = %i\n", SO_TIMESTAMPING);
  printf("SO_RXQ_OVFL = %i\n", SO_RXQ_OVFL);
  printf("SO_WIFI_STATUS = %i\n", SO_WIFI_STATUS);
  printf("SO_PEEK_OFF = %i\n", SO_PEEK_OFF);
  printf("SO_NOFCS = %i\n", SO_NOFCS);
  printf("SO_LOCK_FILTER = %i\n", SO_LOCK_FILTER);
  printf("SO_SELECT_ERR_QUEUE = %i\n", SO_SELECT_ERR_QUEUE);
  printf("SO_BUSY_POLL = %i\n", SO_BUSY_POLL);
  printf("SO_MAX_PACING_RATE = %i\n", SO_MAX_PACING_RATE);
  printf("SO_BPF_EXTENSIONS = %i\n", SO_BPF_EXTENSIONS);
  printf("SO_SNDBUFFORCE = %i\n", SO_SNDBUFFORCE);
  printf("SO_RCVBUFFORCE = %i\n", SO_RCVBUFFORCE);
  printf("SO_RCVLOWAT = %i\n", SO_RCVLOWAT);
  printf("SO_SNDLOWAT = %i\n", SO_SNDLOWAT);
  printf("SO_RCVTIMEO = %i\n", SO_RCVTIMEO);
  printf("SO_SNDTIMEO = %i\n", SO_SNDTIMEO);
  printf("SO_ACCEPTCONN = %i\n", SO_ACCEPTCONN);
  printf("SO_PROTOCOL = %i\n", SO_PROTOCOL);
  printf("SO_DOMAIN = %i\n", SO_DOMAIN);
  printf("SO_INCOMING_CPU = %i\n", SO_INCOMING_CPU);
  printf("SO_ATTACH_BPF = %i\n", SO_ATTACH_BPF);
  printf("SO_DETACH_BPF = %i\n", SO_DETACH_BPF);
  printf("SOL_SOCKET = %i\n", SOL_SOCKET);
  printf("SOL_IPV6 = %i\n", SOL_IPV6);
  printf("SOL_ICMPV6 = %i\n", SOL_ICMPV6);
  printf("SOL_RAW = %i\n", SOL_RAW);
  printf("SOL_DECNET = %i\n", SOL_DECNET);
  printf("SOL_PACKET = %i\n", SOL_PACKET);
  printf("SOL_ATM = %i\n", SOL_ATM);
  printf("SOL_IRDA = %i\n", SOL_IRDA);
  printf("AF_UNSPEC = %i\n", AF_UNSPEC);
  printf("AF_UNIX = %i\n", AF_UNIX);
  printf("AF_LOCAL = %i\n", AF_LOCAL);
  printf("AF_INET = %i\n", AF_INET);
  printf("AF_AX25 = %i\n", AF_AX25);
  printf("AF_IPX = %i\n", AF_IPX);
  printf("AF_APPLETALK = %i\n", AF_APPLETALK);
  printf("AF_NETROM = %i\n", AF_NETROM);
  printf("AF_BRIDGE = %i\n", AF_BRIDGE);
  printf("AF_ATMPVC = %i\n", AF_ATMPVC);
  printf("AF_X25 = %i\n", AF_X25);
  printf("AF_INET6 = %i\n", AF_INET6);
  printf("AF_ROSE = %i\n", AF_ROSE);
  printf("AF_DECnet = %i\n", AF_DECnet);
  printf("AF_NETBEUI = %i\n", AF_NETBEUI);
  printf("AF_SECURITY = %i\n", AF_SECURITY);
  printf("AF_KEY = %i\n", AF_KEY);
  printf("AF_NETLINK = %i\n", AF_NETLINK);
  printf("AF_ROUTE = %i\n", AF_ROUTE);
  printf("AF_PACKET = %i\n", AF_PACKET);
  printf("AF_ASH = %i\n", AF_ASH);
  printf("AF_ECONET = %i\n", AF_ECONET);
  printf("AF_ATMSVC = %i\n", AF_ATMSVC);
  printf("AF_SNA = %i\n", AF_SNA);
  printf("AF_IRDA = %i\n", AF_IRDA);
  printf("AF_PPPOX = %i\n", AF_PPPOX);
  printf("AF_WANPIPE = %i\n", AF_WANPIPE);
  printf("AF_LLC = %i\n", AF_LLC);
  printf("AF_IB = %i\n", AF_IB);
  printf("AF_MPLS = %i\n", AF_MPLS);
  printf("AF_CAN = %i\n", AF_CAN);
  printf("AF_TIPC = %i\n", AF_TIPC);
  printf("AF_BLUETOOTH = %i\n", AF_BLUETOOTH);
  printf("AF_IUCV = %i\n", AF_IUCV);
  printf("AF_RXRPC = %i\n", AF_RXRPC);
  printf("AF_ISDN = %i\n", AF_ISDN);
  printf("AF_PHONET = %i\n", AF_PHONET);
  printf("AF_IEEE802154 = %i\n", AF_IEEE802154);
  printf("AF_CAIF = %i\n", AF_CAIF);
  printf("AF_ALG = %i\n", AF_ALG);
  printf("AF_NFC = %i\n", AF_NFC);
  printf("AF_VSOCK = %i\n", AF_VSOCK);
  printf("AF_MAX = %i\n", AF_MAX);
  printf("PF_UNSPEC = %i\n", PF_UNSPEC);
  printf("PF_UNIX = %i\n", PF_UNIX);
  printf("PF_LOCAL = %i\n", PF_LOCAL);
  printf("PF_INET = %i\n", PF_INET);
  printf("PF_AX25 = %i\n", PF_AX25);
  printf("PF_IPX = %i\n", PF_IPX);
  printf("PF_APPLETALK = %i\n", PF_APPLETALK);
  printf("PF_NETROM = %i\n", PF_NETROM);
  printf("PF_BRIDGE = %i\n", PF_BRIDGE);
  printf("PF_ATMPVC = %i\n", PF_ATMPVC);
  printf("PF_X25 = %i\n", PF_X25);
  printf("PF_INET6 = %i\n", PF_INET6);
  printf("PF_ROSE = %i\n", PF_ROSE);
  printf("PF_DECnet = %i\n", PF_DECnet);
  printf("PF_NETBEUI = %i\n", PF_NETBEUI);
  printf("PF_SECURITY = %i\n", PF_SECURITY);
  printf("PF_KEY = %i\n", PF_KEY);
  printf("PF_NETLINK = %i\n", PF_NETLINK);
  printf("PF_ROUTE = %i\n", PF_ROUTE);
  printf("PF_PACKET = %i\n", PF_PACKET);
  printf("PF_ASH = %i\n", PF_ASH);
  printf("PF_ECONET = %i\n", PF_ECONET);
  printf("PF_ATMSVC = %i\n", PF_ATMSVC);
  printf("PF_SNA = %i\n", PF_SNA);
  printf("PF_IRDA = %i\n", PF_IRDA);
  printf("PF_PPPOX = %i\n", PF_PPPOX);
  printf("PF_WANPIPE = %i\n", PF_WANPIPE);
  printf("PF_LLC = %i\n", PF_LLC);
  printf("PF_IB = %i\n", PF_IB);
  printf("PF_MPLS = %i\n", PF_MPLS);
  printf("PF_CAN = %i\n", PF_CAN);
  printf("PF_TIPC = %i\n", PF_TIPC);
  printf("PF_BLUETOOTH = %i\n", PF_BLUETOOTH);
  printf("PF_IUCV = %i\n", PF_IUCV);
  printf("PF_RXRPC = %i\n", PF_RXRPC);
  printf("PF_ISDN = %i\n", PF_ISDN);
  printf("PF_PHONET = %i\n", PF_PHONET);
  printf("PF_IEEE802154 = %i\n", PF_IEEE802154);
  printf("PF_CAIF = %i\n", PF_CAIF);
  printf("PF_ALG = %i\n", PF_ALG);
  printf("PF_NFC = %i\n", PF_NFC);
  printf("PF_VSOCK = %i\n", PF_VSOCK);
  printf("PF_MAX = %i\n", PF_MAX);
  printf("SOCK_NONBLOCK = %i\n", SOCK_NONBLOCK);
  printf("SOCK_CLOEXEC = %i\n", SOCK_CLOEXEC);
  printf("SOCK_DGRAM = %i\n", SOCK_DGRAM);
  printf("SOCK_STREAM = %i\n", SOCK_STREAM);
  printf("SOCK_RAW = %i\n", SOCK_RAW);
  printf("SOCK_RDM = %i\n", SOCK_RDM);
  printf("SOCK_SEQPACKET = %i\n", SOCK_SEQPACKET);
  printf("SOCK_DCCP = %i\n", SOCK_DCCP);
  printf("SOCK_PACKET = %i\n", SOCK_PACKET);
  printf("IPPROTO_IP = %i\n", IPPROTO_IP);
  printf("IPPROTO_ICMP = %i\n", IPPROTO_ICMP);
  printf("IPPROTO_IGMP = %i\n", IPPROTO_IGMP);
  printf("IPPROTO_IPIP = %i\n", IPPROTO_IPIP);
  printf("IPPROTO_TCP = %i\n", IPPROTO_TCP);
  printf("IPPROTO_EGP = %i\n", IPPROTO_EGP);
  printf("IPPROTO_PUP = %i\n", IPPROTO_PUP);
  printf("IPPROTO_UDP = %i\n", IPPROTO_UDP);
  printf("IPPROTO_IDP = %i\n", IPPROTO_IDP);
  printf("IPPROTO_RSVP = %i\n", IPPROTO_RSVP);
  printf("IPPROTO_GRE = %i\n", IPPROTO_GRE);
  printf("IPPROTO_IPV6 = %i\n", IPPROTO_IPV6);
  printf("IPPROTO_PIM = %i\n", IPPROTO_PIM);
  printf("IPPROTO_ESP = %i\n", IPPROTO_ESP);
  printf("IPPROTO_AH = %i\n", IPPROTO_AH);
  printf("IPPROTO_COMP = %i\n", IPPROTO_COMP);
  printf("IPPROTO_SCTP = %i\n", IPPROTO_SCTP);
  printf("IPPROTO_UDPLITE = %i\n", IPPROTO_UDPLITE);
  printf("IPPROTO_RAW = %i\n", IPPROTO_RAW);
  printf("IPPROTO_HOPOPTS = %i\n", IPPROTO_HOPOPTS);
  printf("IPPROTO_ROUTING = %i\n", IPPROTO_ROUTING);
  printf("IPPROTO_FRAGMENT = %i\n", IPPROTO_FRAGMENT);
  printf("IPPROTO_ICMPV6 = %i\n", IPPROTO_ICMPV6);
  printf("IPPROTO_NONE = %i\n", IPPROTO_NONE);
  printf("IPPROTO_DSTOPTS = %i\n", IPPROTO_DSTOPTS);
  printf("POLLIN = %i\n", POLLIN);
  printf("POLLPRI = %i\n", POLLPRI);
  printf("POLLOUT = %i\n", POLLOUT);
  printf("POLLERR = %i\n", POLLERR);
  printf("POLLHUP = %i\n", POLLHUP);
  printf("POLLNVAL = %i\n", POLLNVAL);
  printf("POLLRDNORM = %i\n", POLLRDNORM);
  printf("POLLRDBAND = %i\n", POLLRDBAND);
  printf("POLLWRBAND = %i\n", POLLWRBAND);
  // printf("POLLMSG = %i\n", POLLMSG);
  // printf("POLLREMOVE = %i\n", POLLREMOVE);
  printf("POLLWRNORM = %i\n", POLLWRNORM);
  printf("O_ASYNC = %i\n", O_ASYNC);
  printf("O_NONBLOCK = %i\n", O_NONBLOCK);
  printf("EAGAIN = %i\n", EAGAIN);
  printf("SHUT_WR = %i\n", SHUT_WR);
  printf("SHUT_RD = %i\n", SHUT_RD);
  // printf("SIG_SETMASK = %i\n", SIG_SETMASK);
  printf("PTRACE_SYSCALL = %i\n", PTRACE_SYSCALL);
  // printf("sizeof(__int128) = %zu\n", sizeof(__int128));
  printf("SIGHUP = %i\n", SIGHUP);
  printf("SIGINT = %i\n", SIGINT);
  printf("SIGQUIT = %i\n", SIGQUIT);
  printf("SIGILL = %i\n", SIGILL);
  printf("SIGTRAP = %i\n", SIGTRAP);
  printf("SIGABRT = %i\n", SIGABRT);
  printf("SIGBUS = %i\n", SIGBUS);
  printf("SIGFPE = %i\n", SIGFPE);
  printf("SIGKILL = %i\n", SIGKILL);
  printf("SIGUSR1 = %i\n", SIGUSR1);
  printf("SIGSEGV = %i\n", SIGSEGV);
  printf("SIGUSR2 = %i\n", SIGUSR2);
  printf("SIGPIPE = %i\n", SIGPIPE);
  printf("SIGALRM = %i\n", SIGALRM);
  printf("SIGTERM = %i\n", SIGTERM);
  printf("SIGSTKFLT = %i\n", SIGSTKFLT);
  printf("SIGCHLD = %i\n", SIGCHLD);
  printf("SIGCONT = %i\n", SIGCONT);
  printf("SIGSTOP = %i\n", SIGSTOP);
  printf("SIGTSTP = %i\n", SIGTSTP);
  printf("SIGTTIN = %i\n", SIGTTIN);
  printf("SIGTTOU = %i\n", SIGTTOU);
  printf("SIGURG = %i\n", SIGURG);
  printf("SIGXCPU = %i\n", SIGXCPU);
  printf("SIGXFSZ = %i\n", SIGXFSZ);
  printf("SIGVTALRM = %i\n", SIGVTALRM);
  printf("SIGPROF = %i\n", SIGPROF);
  printf("SIGWINCH = %i\n", SIGWINCH);
  printf("SIGIO = %i\n", SIGIO);
  printf("SIGPWR = %i\n", SIGPWR);
  printf("SIGSYS = %i\n", SIGSYS);
  printf("#define IN_ACCESS 0x%x\n", IN_ACCESS);
  printf("#define IN_MODIFY 0x%x\n", IN_MODIFY);
  printf("#define IN_ATTRIB 0x%x\n", IN_ATTRIB);
  printf("#define IN_CLOSE_WRITE 0x%x\n", IN_CLOSE_WRITE);
  printf("#define IN_CLOSE_NOWRITE 0x%x\n", IN_CLOSE_NOWRITE);
  printf("#define IN_CLOSE 0x%x\n", IN_CLOSE);
  printf("#define IN_OPEN 0x%x\n", IN_OPEN);
  printf("#define IN_MOVED_FROM 0x%x\n", IN_MOVED_FROM);
  printf("#define IN_MOVED_TO 0x%x\n", IN_MOVED_TO);
  printf("#define IN_MOVE 0x%x\n", IN_MOVE);
  printf("#define IN_CREATE 0x%x\n", IN_CREATE);
  printf("#define IN_DELETE 0x%x\n", IN_DELETE);
  printf("#define IN_DELETE_SELF 0x%x\n", IN_DELETE_SELF);
  printf("#define IN_MOVE_SELF 0x%x\n", IN_MOVE_SELF);
  printf("#define IN_UNMOUNT 0x%x\n", IN_UNMOUNT);
  printf("#define IN_Q_OVERFLOW 0x%x\n", IN_Q_OVERFLOW);
  printf("#define IN_IGNORED 0x%x\n", IN_IGNORED);
  printf("#define IN_CLOSE 0x%x\n", IN_CLOSE);
  printf("#define IN_MOVE 0x%x\n", IN_MOVE);
  printf("#define IN_ONLYDIR 0x%x\n", IN_ONLYDIR);
  printf("#define IN_DONT_FOLLOW 0x%x\n", IN_DONT_FOLLOW);
  printf("#define IN_EXCL_UNLINK 0x%x\n", IN_EXCL_UNLINK);
  printf("#define IN_MASK_CREATE 0x%x\n", IN_MASK_CREATE);
  printf("#define IN_MASK_ADD 0x%x\n", IN_MASK_ADD);
  printf("#define IN_ISDIR 0x%x\n", IN_ISDIR);
  printf("#define IN_ONESHOT 0x%x\n", IN_ONESHOT);
  printf("#define IN_ALL_EVENTS 0x%x\n", IN_ALL_EVENTS);

  printf("BUFSIZ = %zu\n", BUFSIZ);
  printf("#ifndef GLOB_ALTDIRFUNC\n#define GLOB_ALTDIRFUNC %d\n#endif\n", GLOB_ALTDIRFUNC);
  printf("#ifndef GLOB_BRACE\n#define GLOB_BRACE %d\n#endif\n", GLOB_BRACE);
  printf("#ifndef GLOB_NOMAGIC\n#define GLOB_NOMAGIC %d\n#endif\n", GLOB_NOMAGIC);
  printf("#ifndef GLOB_TILDE\n#define GLOB_TILDE %d\n#endif\n", GLOB_TILDE);
  printf("#ifndef GLOB_TILDE_CHECK\n#define GLOB_TILDE_CHECK %d\n#endif\n", GLOB_TILDE_CHECK);
  printf("#ifndef GLOB_ONLYDIR\n#define GLOB_ONLYDIR %d\n#endif\n", GLOB_ONLYDIR);
  printf("#ifndef GLOB_MAGCHAR\n#define GLOB_MAGCHAR %d\n#endif\n", GLOB_MAGCHAR);
  printf("#define _SC_OPEN_MAX 0x%x\n", _SC_OPEN_MAX);
  printf("sizeof(PmDeviceInfo) = %zu\n", sizeof(PmDeviceInfo));
  printf("sizeof(PmEvent) = %zu\n", sizeof(PmEvent));
  printf("pmNoError = %i\n", pmNoError);
  printf("pmNoData = %i\n", pmNoData);
  printf("pmGotData = %i\n", pmGotData);
  printf("pmHostError = %i\n", pmHostError);
  printf("pmInvalidDeviceId = %i\n", pmInvalidDeviceId);
  printf("pmInsufficientMemory = %i\n", pmInsufficientMemory);
  printf("pmBufferTooSmall = %i\n", pmBufferTooSmall);
  printf("pmBufferOverflow = %i\n", pmBufferOverflow);
  printf("pmBadPtr = %i\n", pmBadPtr);
  printf("pmBadData = %i\n", pmBadData);
  printf("pmInternalError = %i\n", pmInternalError);
  printf("pmBufferMaxSize = %i\n", pmBufferMaxSize);
  printf("pmNotImplemented = %i\n", pmNotImplemented);
  printf("pmInterfaceNotSupported = %i\n", pmInterfaceNotSupported);
  printf("pmNameConflict = %i\n", pmNameConflict);

  printf("sizeof(struct inotify_event) = %zu\n", sizeof(struct inotify_event));
  printf("IN_ACCESS = %i\n", IN_ACCESS);
  printf("IN_MODIFY = %i\n", IN_MODIFY);
  printf("IN_ATTRIB = %i\n", IN_ATTRIB);
  printf("IN_CLOSE_WRITE = %i\n", IN_CLOSE_WRITE);
  printf("IN_CLOSE_NOWRITE = %i\n", IN_CLOSE_NOWRITE);
  printf("IN_CLOSE = %i\n", IN_CLOSE);
  printf("IN_OPEN = %i\n", IN_OPEN);
  printf("IN_MOVED_FROM = %i\n", IN_MOVED_FROM);
  printf("IN_MOVED_TO = %i\n", IN_MOVED_TO);
  printf("IN_MOVE = %i\n", IN_MOVE);
  printf("IN_CREATE = %i\n", IN_CREATE);
  printf("IN_DELETE = %i\n", IN_DELETE);
  printf("IN_DELETE_SELF = %i\n", IN_DELETE_SELF);
  printf("IN_MOVE_SELF = %i\n", IN_MOVE_SELF);
  printf("IN_UNMOUNT = %i\n", IN_UNMOUNT);
  printf("IN_Q_OVERFLOW = %i\n", IN_Q_OVERFLOW);
  printf("IN_IGNORED = %i\n", IN_IGNORED);
  printf("IN_ONLYDIR = %i\n", IN_ONLYDIR);
  printf("IN_DONT_FOLLOW = %i\n", IN_DONT_FOLLOW);
  printf("IN_EXCL_UNLINK = %i\n", IN_EXCL_UNLINK);
  printf("IN_MASK_ADD = %i\n", IN_MASK_ADD);
  printf("IN_ISDIR = %i\n", IN_ISDIR);
  printf("IN_ONESHOT = %i\n", IN_ONESHOT);
  printf("RNDGETENTCNT = %x\n", RNDGETENTCNT);
  printf("sizeof(z_stream) = %zu\n", sizeof(z_stream));
  printf("sizeof(gz_header) = %zu\n", sizeof(gz_header));

  z_stream svar;
  printf("\nz_stream %zu\n", sizeof(svar));
  printf(".next_in %zu %zu\n", offsetof(z_stream, next_in), sizeof(svar.next_in));
  printf(".avail_in %zu %zu\n", offsetof(z_stream, avail_in), sizeof(svar.avail_in));
  printf(".total_in %zu %zu\n", offsetof(z_stream, total_in), sizeof(svar.total_in));
  printf(".next_out %zu %zu\n", offsetof(z_stream, next_out), sizeof(svar.next_out));
  printf(".avail_out %zu %zu\n", offsetof(z_stream, avail_out), sizeof(svar.avail_out));
  printf(".total_out %zu %zu\n", offsetof(z_stream, total_out), sizeof(svar.total_out));
  printf(".msg %zu %zu\n", offsetof(z_stream, msg), sizeof(svar.msg));
  printf(".state %zu %zu\n", offsetof(z_stream, state), sizeof(svar.state));
  printf(".zalloc %zu %zu\n", offsetof(z_stream, zalloc), sizeof(svar.zalloc));
  printf(".zfree %zu %zu\n", offsetof(z_stream, zfree), sizeof(svar.zfree));
  printf(".opaque %zu %zu\n", offsetof(z_stream, opaque), sizeof(svar.opaque));
  printf(".data_type %zu %zu\n", offsetof(z_stream, data_type), sizeof(svar.data_type));
  printf(".adler %zu %zu\n", offsetof(z_stream, adler), sizeof(svar.adler));
  printf(".reserved %zu %zu\n", offsetof(z_stream, reserved), sizeof(svar.reserved));

  printf("LWS_CALLBACK_LOCK_POLL = %i\n", LWS_CALLBACK_LOCK_POLL);
  printf("LWS_CALLBACK_UNLOCK_POLL = %i\n", LWS_CALLBACK_UNLOCK_POLL);
  printf("LWS_CALLBACK_ADD_POLL_FD = %i\n", LWS_CALLBACK_ADD_POLL_FD);
  printf("LWS_CALLBACK_DEL_POLL_FD = %i\n", LWS_CALLBACK_DEL_POLL_FD);
  printf("LWS_CALLBACK_CHANGE_MODE_POLL_FD = %i\n", LWS_CALLBACK_CHANGE_MODE_POLL_FD);

  return 0;
}

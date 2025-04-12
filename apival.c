#define _GNU_SOURCE 1
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
#include <fcntl.h>
#include <fnmatch.h>
//#include <dns.h>
#include "quickjs/qjs-net/libwebsockets/include/libwebsockets.h"
#include <mariadb/mysql.h>

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

  for(size_t i = 0; i < 256; i++)
    map[i] = escape_char_pred(i);
  for(size_t i = 0; i < 256; i++) {
    printf("%s0x%02x", i > 0 ? ", " : "", map[i]);
  }
  // printf("PATH_MAX = 0o%o\n", PATH_MAX);
  // printf("LWS_PRE = 0o%o\n", LWS_PRE);
  // printf("NSIG = 0o%o\n", NSIG);
  // printf("SIGRTMAX = 0o%o\n", SIGRTMAX);
  // printf("SIGRTMIN = 0o%o\n", SIGRTMIN);
  // printf("sizeof(dev_t) = %zu\n", sizeof(dev_t));
  // printf("sizeof(mode_t) = %zu\n", sizeof(mode_t));
  // printf("sizeof(socklen_t) = %zu\n", sizeof(socklen_t));
  // printf("sizeof(sa_family_t) = %zu\n", sizeof(sa_family_t));
  // printf("sizeof(struct timeval) = %zu\n", sizeof(struct timeval));
  struct timeval tv;
  // printf("sizeof(tv.tv_sec) = %zu\n", sizeof(tv.tv_sec));
  // printf("sizeof(struct timespec) = %zu\n", sizeof(struct timespec));
  // printf("SOCK_STREAM = 0o%o\n", SOCK_STREAM);
  // printf("SOCK_DGRAM = 0o%o\n", SOCK_DGRAM);
  printf("SHUT_RD = 0o%o\n", SHUT_RD);
  printf("SHUT_WR = 0o%o\n", SHUT_WR);
  printf("SHUT_RDWR = 0o%o\n", SHUT_RDWR);
  printf("SO_ERROR = 0o%o\n", SO_ERROR);
  printf("SO_DEBUG = 0o%o\n", SO_DEBUG);
  printf("SO_REUSEADDR = 0o%o\n", SO_REUSEADDR);
  printf("SO_KEEPALIVE = 0o%o\n", SO_KEEPALIVE);
  printf("SO_DONTROUTE = 0o%o\n", SO_DONTROUTE);
  printf("SO_BROADCAST = 0o%o\n", SO_BROADCAST);
  printf("SO_OOBINLINE = 0o%o\n", SO_OOBINLINE);
  printf("SO_REUSEPORT = 0o%o\n", SO_REUSEPORT);
  printf("SO_SNDBUF = 0o%o\n", SO_SNDBUF);
  printf("SO_RCVBUF = 0o%o\n", SO_RCVBUF);
  printf("SO_NO_CHECK = 0o%o\n", SO_NO_CHECK);
  printf("SO_PRIORITY = 0o%o\n", SO_PRIORITY);
  printf("SO_BSDCOMPAT = 0o%o\n", SO_BSDCOMPAT);
  printf("SO_PASSCRED = 0o%o\n", SO_PASSCRED);
  printf("SO_PEERCRED = 0o%o\n", SO_PEERCRED);
  printf("SO_SECURITY_AUTHENTICATION = 0o%o\n", SO_SECURITY_AUTHENTICATION);
  printf("SO_SECURITY_ENCRYPTION_TRANSPORT = 0o%o\n", SO_SECURITY_ENCRYPTION_TRANSPORT);
  printf("SO_SECURITY_ENCRYPTION_NETWORK = 0o%o\n", SO_SECURITY_ENCRYPTION_NETWORK);
  printf("SO_BINDTODEVICE = 0o%o\n", SO_BINDTODEVICE);
  printf("SO_ATTACH_FILTER = 0o%o\n", SO_ATTACH_FILTER);
  printf("SO_DETACH_FILTER = 0o%o\n", SO_DETACH_FILTER);
  printf("SO_GET_FILTER = 0o%o\n", SO_GET_FILTER);
  printf("SO_PEERNAME = 0o%o\n", SO_PEERNAME);
  printf("SO_TIMESTAMP = 0o%o\n", SO_TIMESTAMP);
  printf("SO_PEERSEC = 0o%o\n", SO_PEERSEC);
  printf("SO_PASSSEC = 0o%o\n", SO_PASSSEC);
  printf("SO_TIMESTAMPNS = 0o%o\n", SO_TIMESTAMPNS);
  printf("SO_MARK = 0o%o\n", SO_MARK);
  printf("SO_TIMESTAMPING = 0o%o\n", SO_TIMESTAMPING);
  printf("SO_RXQ_OVFL = 0o%o\n", SO_RXQ_OVFL);
  printf("SO_WIFI_STATUS = 0o%o\n", SO_WIFI_STATUS);
  printf("SO_PEEK_OFF = 0o%o\n", SO_PEEK_OFF);
  printf("SO_NOFCS = 0o%o\n", SO_NOFCS);
  printf("SO_LOCK_FILTER = 0o%o\n", SO_LOCK_FILTER);
  printf("SO_SELECT_ERR_QUEUE = 0o%o\n", SO_SELECT_ERR_QUEUE);
  printf("SO_BUSY_POLL = 0o%o\n", SO_BUSY_POLL);
  printf("SO_MAX_PACING_RATE = 0o%o\n", SO_MAX_PACING_RATE);
  printf("SO_BPF_EXTENSIONS = 0o%o\n", SO_BPF_EXTENSIONS);
  printf("SO_SNDBUFFORCE = 0o%o\n", SO_SNDBUFFORCE);
  printf("SO_RCVBUFFORCE = 0o%o\n", SO_RCVBUFFORCE);
  printf("SO_RCVLOWAT = 0o%o\n", SO_RCVLOWAT);
  printf("SO_SNDLOWAT = 0o%o\n", SO_SNDLOWAT);
  printf("SO_RCVTIMEO = 0o%o\n", SO_RCVTIMEO);
  printf("SO_SNDTIMEO = 0o%o\n", SO_SNDTIMEO);
  printf("SO_ACCEPTCONN = 0o%o\n", SO_ACCEPTCONN);
  printf("SO_PROTOCOL = 0o%o\n", SO_PROTOCOL);
  printf("SO_DOMAIN = 0o%o\n", SO_DOMAIN);
  printf("SO_INCOMING_CPU = 0o%o\n", SO_INCOMING_CPU);
  printf("SO_ATTACH_BPF = 0o%o\n", SO_ATTACH_BPF);
  printf("SO_DETACH_BPF = 0o%o\n", SO_DETACH_BPF);
  printf("SOL_SOCKET = 0o%o\n", SOL_SOCKET);
  printf("SOL_IPV6 = 0o%o\n", SOL_IPV6);
  printf("SOL_ICMPV6 = 0o%o\n", SOL_ICMPV6);
  printf("SOL_RAW = 0o%o\n", SOL_RAW);
  printf("SOL_DECNET = 0o%o\n", SOL_DECNET);
  printf("SOL_PACKET = 0o%o\n", SOL_PACKET);
  printf("SOL_ATM = 0o%o\n", SOL_ATM);
  printf("SOL_IRDA = 0o%o\n", SOL_IRDA);
  printf("AF_UNSPEC = 0o%o\n", AF_UNSPEC);
  printf("AF_UNIX = 0o%o\n", AF_UNIX);
  printf("AF_LOCAL = 0o%o\n", AF_LOCAL);
  printf("AF_INET = 0o%o\n", AF_INET);
  printf("AF_AX25 = 0o%o\n", AF_AX25);
  printf("AF_IPX = 0o%o\n", AF_IPX);
  printf("AF_APPLETALK = 0o%o\n", AF_APPLETALK);
  printf("AF_NETROM = 0o%o\n", AF_NETROM);
  printf("AF_BRIDGE = 0o%o\n", AF_BRIDGE);
  printf("AF_ATMPVC = 0o%o\n", AF_ATMPVC);
  printf("AF_X25 = 0o%o\n", AF_X25);
  printf("AF_INET6 = 0o%o\n", AF_INET6);
  printf("AF_ROSE = 0o%o\n", AF_ROSE);
  printf("AF_DECnet = 0o%o\n", AF_DECnet);
  printf("AF_NETBEUI = 0o%o\n", AF_NETBEUI);
  printf("AF_SECURITY = 0o%o\n", AF_SECURITY);
  printf("AF_KEY = 0o%o\n", AF_KEY);
  printf("AF_NETLINK = 0o%o\n", AF_NETLINK);
  printf("AF_ROUTE = 0o%o\n", AF_ROUTE);
  printf("AF_PACKET = 0o%o\n", AF_PACKET);
  printf("AF_ASH = 0o%o\n", AF_ASH);
  printf("AF_ECONET = 0o%o\n", AF_ECONET);
  printf("AF_ATMSVC = 0o%o\n", AF_ATMSVC);
  printf("AF_SNA = 0o%o\n", AF_SNA);
  printf("AF_IRDA = 0o%o\n", AF_IRDA);
  printf("AF_PPPOX = 0o%o\n", AF_PPPOX);
  printf("AF_WANPIPE = 0o%o\n", AF_WANPIPE);
  printf("AF_LLC = 0o%o\n", AF_LLC);
  printf("AF_IB = 0o%o\n", AF_IB);
  printf("AF_MPLS = 0o%o\n", AF_MPLS);
  printf("AF_CAN = 0o%o\n", AF_CAN);
  printf("AF_TIPC = 0o%o\n", AF_TIPC);
  printf("AF_BLUETOOTH = 0o%o\n", AF_BLUETOOTH);
  printf("AF_IUCV = 0o%o\n", AF_IUCV);
  printf("AF_RXRPC = 0o%o\n", AF_RXRPC);
  printf("AF_ISDN = 0o%o\n", AF_ISDN);
  printf("AF_PHONET = 0o%o\n", AF_PHONET);
  printf("AF_IEEE802154 = 0o%o\n", AF_IEEE802154);
  printf("AF_CAIF = 0o%o\n", AF_CAIF);
  printf("AF_ALG = 0o%o\n", AF_ALG);
  printf("AF_NFC = 0o%o\n", AF_NFC);
  printf("AF_VSOCK = 0o%o\n", AF_VSOCK);
  printf("AF_MAX = 0o%o\n", AF_MAX);
  printf("PF_UNSPEC = 0o%o\n", PF_UNSPEC);
  printf("PF_UNIX = 0o%o\n", PF_UNIX);
  printf("PF_LOCAL = 0o%o\n", PF_LOCAL);
  printf("PF_INET = 0o%o\n", PF_INET);
  printf("PF_AX25 = 0o%o\n", PF_AX25);
  printf("PF_IPX = 0o%o\n", PF_IPX);
  printf("PF_APPLETALK = 0o%o\n", PF_APPLETALK);
  printf("PF_NETROM = 0o%o\n", PF_NETROM);
  printf("PF_BRIDGE = 0o%o\n", PF_BRIDGE);
  printf("PF_ATMPVC = 0o%o\n", PF_ATMPVC);
  printf("PF_X25 = 0o%o\n", PF_X25);
  printf("PF_INET6 = 0o%o\n", PF_INET6);
  printf("PF_ROSE = 0o%o\n", PF_ROSE);
  printf("PF_DECnet = 0o%o\n", PF_DECnet);
  printf("PF_NETBEUI = 0o%o\n", PF_NETBEUI);
  printf("PF_SECURITY = 0o%o\n", PF_SECURITY);
  printf("PF_KEY = 0o%o\n", PF_KEY);
  printf("PF_NETLINK = 0o%o\n", PF_NETLINK);
  printf("PF_ROUTE = 0o%o\n", PF_ROUTE);
  printf("PF_PACKET = 0o%o\n", PF_PACKET);
  printf("PF_ASH = 0o%o\n", PF_ASH);
  printf("PF_ECONET = 0o%o\n", PF_ECONET);
  printf("PF_ATMSVC = 0o%o\n", PF_ATMSVC);
  printf("PF_SNA = 0o%o\n", PF_SNA);
  printf("PF_IRDA = 0o%o\n", PF_IRDA);
  printf("PF_PPPOX = 0o%o\n", PF_PPPOX);
  printf("PF_WANPIPE = 0o%o\n", PF_WANPIPE);
  printf("PF_LLC = 0o%o\n", PF_LLC);
  printf("PF_IB = 0o%o\n", PF_IB);
  printf("PF_MPLS = 0o%o\n", PF_MPLS);
  printf("PF_CAN = 0o%o\n", PF_CAN);
  printf("PF_TIPC = 0o%o\n", PF_TIPC);
  printf("PF_BLUETOOTH = 0o%o\n", PF_BLUETOOTH);
  printf("PF_IUCV = 0o%o\n", PF_IUCV);
  printf("PF_RXRPC = 0o%o\n", PF_RXRPC);
  printf("PF_ISDN = 0o%o\n", PF_ISDN);
  printf("PF_PHONET = 0o%o\n", PF_PHONET);
  printf("PF_IEEE802154 = 0o%o\n", PF_IEEE802154);
  printf("PF_CAIF = 0o%o\n", PF_CAIF);
  printf("PF_ALG = 0o%o\n", PF_ALG);
  printf("PF_NFC = 0o%o\n", PF_NFC);
  printf("PF_VSOCK = 0o%o\n", PF_VSOCK);
  printf("PF_MAX = 0o%o\n", PF_MAX);
  printf("SOCK_NONBLOCK = 0o%o\n", SOCK_NONBLOCK);
  printf("SOCK_CLOEXEC = 0o%o\n", SOCK_CLOEXEC);
  printf("SOCK_DGRAM = 0o%o\n", SOCK_DGRAM);
  printf("SOCK_STREAM = 0o%o\n", SOCK_STREAM);
  printf("SOCK_RAW = 0o%o\n", SOCK_RAW);
  printf("SOCK_RDM = 0o%o\n", SOCK_RDM);
  printf("SOCK_SEQPACKET = 0o%o\n", SOCK_SEQPACKET);
  printf("SOCK_DCCP = 0o%o\n", SOCK_DCCP);
  printf("SOCK_PACKET = 0o%o\n", SOCK_PACKET);
  printf("IPPROTO_IP = 0o%o\n", IPPROTO_IP);
  printf("IPPROTO_ICMP = 0o%o\n", IPPROTO_ICMP);
  printf("IPPROTO_IGMP = 0o%o\n", IPPROTO_IGMP);
  printf("IPPROTO_IPIP = 0o%o\n", IPPROTO_IPIP);
  printf("IPPROTO_TCP = 0o%o\n", IPPROTO_TCP);
  printf("IPPROTO_EGP = 0o%o\n", IPPROTO_EGP);
  printf("IPPROTO_PUP = 0o%o\n", IPPROTO_PUP);
  printf("IPPROTO_UDP = 0o%o\n", IPPROTO_UDP);
  printf("IPPROTO_IDP = 0o%o\n", IPPROTO_IDP);
  printf("IPPROTO_RSVP = 0o%o\n", IPPROTO_RSVP);
  printf("IPPROTO_GRE = 0o%o\n", IPPROTO_GRE);
  printf("IPPROTO_IPV6 = 0o%o\n", IPPROTO_IPV6);
  printf("IPPROTO_PIM = 0o%o\n", IPPROTO_PIM);
  printf("IPPROTO_ESP = 0o%o\n", IPPROTO_ESP);
  printf("IPPROTO_AH = 0o%o\n", IPPROTO_AH);
  printf("IPPROTO_COMP = 0o%o\n", IPPROTO_COMP);
  printf("IPPROTO_SCTP = 0o%o\n", IPPROTO_SCTP);
  printf("IPPROTO_UDPLITE = 0o%o\n", IPPROTO_UDPLITE);
  printf("IPPROTO_RAW = 0o%o\n", IPPROTO_RAW);
  printf("IPPROTO_HOPOPTS = 0o%o\n", IPPROTO_HOPOPTS);
  printf("IPPROTO_ROUTING = 0o%o\n", IPPROTO_ROUTING);
  printf("IPPROTO_FRAGMENT = 0o%o\n", IPPROTO_FRAGMENT);
  printf("IPPROTO_ICMPV6 = 0o%o\n", IPPROTO_ICMPV6);
  printf("IPPROTO_NONE = 0o%o\n", IPPROTO_NONE);
  printf("IPPROTO_DSTOPTS = 0o%o\n", IPPROTO_DSTOPTS);
  printf("POLLIN = 0o%o\n", POLLIN);
  printf("POLLPRI = 0o%o\n", POLLPRI);
  printf("POLLOUT = 0o%o\n", POLLOUT);
  printf("POLLERR = 0o%o\n", POLLERR);
  printf("POLLHUP = 0o%o\n", POLLHUP);
  printf("POLLNVAL = 0o%o\n", POLLNVAL);
  printf("POLLRDNORM = 0o%o\n", POLLRDNORM);
  printf("POLLRDBAND = 0o%o\n", POLLRDBAND);
  printf("POLLWRBAND = 0o%o\n", POLLWRBAND);
  // printf("POLLMSG = 0o%o\n", POLLMSG);
  // printf("POLLREMOVE = 0o%o\n", POLLREMOVE);
  printf("POLLWRNORM = 0o%o\n", POLLWRNORM);
  printf("EAGAIN = 0o%o\n", EAGAIN);
  printf("SHUT_WR = 0o%o\n", SHUT_WR);
  printf("SHUT_RD = 0o%o\n", SHUT_RD);
  // printf("SIG_SETMASK = 0o%o\n", SIG_SETMASK);
  printf("PTRACE_SYSCALL = 0o%o\n", PTRACE_SYSCALL);
  // printf("sizeof(__int128) = %zu\n", sizeof(__int128));
  printf("SIGHUP = %d\n", SIGHUP);
  printf("SIGINT = %d\n", SIGINT);
  printf("SIGQUIT = %d\n", SIGQUIT);
  printf("SIGILL = %d\n", SIGILL);
  printf("SIGTRAP = %d\n", SIGTRAP);
  printf("SIGABRT = %d\n", SIGABRT);
  printf("SIGBUS = %d\n", SIGBUS);
  printf("SIGFPE = %d\n", SIGFPE);
  printf("SIGKILL = %d\n", SIGKILL);
  printf("SIGUSR1 = %d\n", SIGUSR1);
  printf("SIGSEGV = %d\n", SIGSEGV);
  printf("SIGUSR2 = %d\n", SIGUSR2);
  printf("SIGPIPE = %d\n", SIGPIPE);
  printf("SIGALRM = %d\n", SIGALRM);
  printf("SIGTERM = %d\n", SIGTERM);
  printf("SIGSTKFLT = %d\n", SIGSTKFLT);
  printf("SIGCHLD = %d\n", SIGCHLD);
  printf("SIGCONT = %d\n", SIGCONT);
  printf("SIGSTOP = %d\n", SIGSTOP);
  printf("SIGTSTP = %d\n", SIGTSTP);
  printf("SIGTTIN = %d\n", SIGTTIN);
  printf("SIGTTOU = %d\n", SIGTTOU);
  printf("SIGURG = %d\n", SIGURG);
  printf("SIGXCPU = %d\n", SIGXCPU);
  printf("SIGXFSZ = %d\n", SIGXFSZ);
  printf("SIGVTALRM = %d\n", SIGVTALRM);
  printf("SIGPROF = %d\n", SIGPROF);
  printf("SIGWINCH = %d\n", SIGWINCH);
  printf("SIGIO = %d\n", SIGIO);
  printf("SIGPWR = %d\n", SIGPWR);
  printf("SIGSYS = %d\n", SIGSYS);
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
  printf("pmNoError = 0o%o\n", pmNoError);
  printf("pmNoData = 0o%o\n", pmNoData);
  printf("pmGotData = 0o%o\n", pmGotData);
  printf("pmHostError = 0o%o\n", pmHostError);
  printf("pmInvalidDeviceId = 0o%o\n", pmInvalidDeviceId);
  printf("pmInsufficientMemory = 0o%o\n", pmInsufficientMemory);
  printf("pmBufferTooSmall = 0o%o\n", pmBufferTooSmall);
  printf("pmBufferOverflow = 0o%o\n", pmBufferOverflow);
  printf("pmBadPtr = 0o%o\n", pmBadPtr);
  printf("pmBadData = 0o%o\n", pmBadData);
  printf("pmInternalError = 0o%o\n", pmInternalError);
  printf("pmBufferMaxSize = 0o%o\n", pmBufferMaxSize);

  printf("sizeof(struct inotify_event) = %zu\n", sizeof(struct inotify_event));
  printf("IN_ACCESS = 0o%o\n", IN_ACCESS);
  printf("IN_MODIFY = 0o%o\n", IN_MODIFY);
  printf("IN_ATTRIB = 0o%o\n", IN_ATTRIB);
  printf("IN_CLOSE_WRITE = 0o%o\n", IN_CLOSE_WRITE);
  printf("IN_CLOSE_NOWRITE = 0o%o\n", IN_CLOSE_NOWRITE);
  printf("IN_CLOSE = 0o%o\n", IN_CLOSE);
  printf("IN_OPEN = 0o%o\n", IN_OPEN);
  printf("IN_MOVED_FROM = 0o%o\n", IN_MOVED_FROM);
  printf("IN_MOVED_TO = 0o%o\n", IN_MOVED_TO);
  printf("IN_MOVE = 0o%o\n", IN_MOVE);
  printf("IN_CREATE = 0o%o\n", IN_CREATE);
  printf("IN_DELETE = 0o%o\n", IN_DELETE);
  printf("IN_DELETE_SELF = 0o%o\n", IN_DELETE_SELF);
  printf("IN_MOVE_SELF = 0o%o\n", IN_MOVE_SELF);
  printf("IN_UNMOUNT = 0o%o\n", IN_UNMOUNT);
  printf("IN_Q_OVERFLOW = 0o%o\n", IN_Q_OVERFLOW);
  printf("IN_IGNORED = 0o%o\n", IN_IGNORED);
  printf("IN_ONLYDIR = 0o%o\n", IN_ONLYDIR);
  printf("IN_DONT_FOLLOW = 0o%o\n", IN_DONT_FOLLOW);
  printf("IN_EXCL_UNLINK = 0o%o\n", IN_EXCL_UNLINK);
  printf("IN_MASK_ADD = 0o%o\n", IN_MASK_ADD);
  printf("IN_ISDIR = 0o%o\n", IN_ISDIR);
  printf("IN_ONESHOT = 0o%o\n", IN_ONESHOT);
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

  printf("LWS_CALLBACK_LOCK_POLL = 0o%o\n", LWS_CALLBACK_LOCK_POLL);
  printf("LWS_CALLBACK_UNLOCK_POLL = 0o%o\n", LWS_CALLBACK_UNLOCK_POLL);
  printf("LWS_CALLBACK_ADD_POLL_FD = 0o%o\n", LWS_CALLBACK_ADD_POLL_FD);
  printf("LWS_CALLBACK_DEL_POLL_FD = 0o%o\n", LWS_CALLBACK_DEL_POLL_FD);
  printf("LWS_CALLBACK_CHANGE_MODE_POLL_FD = 0o%o\n", LWS_CALLBACK_CHANGE_MODE_POLL_FD);

  printf("WSI_TOKEN_GET_URI = 0o%o\n", WSI_TOKEN_GET_URI);
  printf("WSI_TOKEN_POST_URI = 0o%o\n", WSI_TOKEN_POST_URI);
  printf("WSI_TOKEN_GET_URI = 0o%o\n", WSI_TOKEN_GET_URI);
  printf("WSI_TOKEN_POST_URI = 0o%o\n", WSI_TOKEN_POST_URI);
  printf("WSI_TOKEN_OPTIONS_URI = 0o%o\n", WSI_TOKEN_OPTIONS_URI);
  printf("WSI_TOKEN_HOST = 0o%o\n", WSI_TOKEN_HOST);
  printf("WSI_TOKEN_CONNECTION = 0o%o\n", WSI_TOKEN_CONNECTION);
  printf("WSI_TOKEN_UPGRADE = 0o%o\n", WSI_TOKEN_UPGRADE);
  printf("WSI_TOKEN_ORIGIN = 0o%o\n", WSI_TOKEN_ORIGIN);
  printf("WSI_TOKEN_DRAFT = 0o%o\n", WSI_TOKEN_DRAFT);
  printf("WSI_TOKEN_CHALLENGE = 0o%o\n", WSI_TOKEN_CHALLENGE);
  printf("WSI_TOKEN_EXTENSIONS = 0o%o\n", WSI_TOKEN_EXTENSIONS);
  printf("WSI_TOKEN_KEY1 = 0o%o\n", WSI_TOKEN_KEY1);
  printf("WSI_TOKEN_KEY2 = 0o%o\n", WSI_TOKEN_KEY2);
  printf("WSI_TOKEN_PROTOCOL = 0o%o\n", WSI_TOKEN_PROTOCOL);
  printf("WSI_TOKEN_ACCEPT = 0o%o\n", WSI_TOKEN_ACCEPT);
  printf("WSI_TOKEN_NONCE = 0o%o\n", WSI_TOKEN_NONCE);
  printf("WSI_TOKEN_HTTP = 0o%o\n", WSI_TOKEN_HTTP);
  printf("WSI_TOKEN_HTTP2_SETTINGS = 0o%o\n", WSI_TOKEN_HTTP2_SETTINGS);
  printf("WSI_TOKEN_HTTP_ACCEPT = 0o%o\n", WSI_TOKEN_HTTP_ACCEPT);
  printf("WSI_TOKEN_HTTP_AC_REQUEST_HEADERS = 0o%o\n", WSI_TOKEN_HTTP_AC_REQUEST_HEADERS);
  printf("WSI_TOKEN_HTTP_IF_MODIFIED_SINCE = 0o%o\n", WSI_TOKEN_HTTP_IF_MODIFIED_SINCE);
  printf("WSI_TOKEN_HTTP_IF_NONE_MATCH = 0o%o\n", WSI_TOKEN_HTTP_IF_NONE_MATCH);
  printf("WSI_TOKEN_HTTP_ACCEPT_ENCODING = 0o%o\n", WSI_TOKEN_HTTP_ACCEPT_ENCODING);
  printf("WSI_TOKEN_HTTP_ACCEPT_LANGUAGE = 0o%o\n", WSI_TOKEN_HTTP_ACCEPT_LANGUAGE);
  printf("WSI_TOKEN_HTTP_PRAGMA = 0o%o\n", WSI_TOKEN_HTTP_PRAGMA);
  printf("WSI_TOKEN_HTTP_CACHE_CONTROL = 0o%o\n", WSI_TOKEN_HTTP_CACHE_CONTROL);
  printf("WSI_TOKEN_HTTP_AUTHORIZATION = 0o%o\n", WSI_TOKEN_HTTP_AUTHORIZATION);
  printf("WSI_TOKEN_HTTP_COOKIE = 0o%o\n", WSI_TOKEN_HTTP_COOKIE);
  printf("WSI_TOKEN_HTTP_CONTENT_LENGTH = 0o%o\n", WSI_TOKEN_HTTP_CONTENT_LENGTH);
  printf("WSI_TOKEN_HTTP_CONTENT_TYPE = 0o%o\n", WSI_TOKEN_HTTP_CONTENT_TYPE);
  printf("WSI_TOKEN_HTTP_DATE = 0o%o\n", WSI_TOKEN_HTTP_DATE);
  printf("WSI_TOKEN_HTTP_RANGE = 0o%o\n", WSI_TOKEN_HTTP_RANGE);
  printf("WSI_TOKEN_HTTP_REFERER = 0o%o\n", WSI_TOKEN_HTTP_REFERER);
  printf("WSI_TOKEN_KEY = 0o%o\n", WSI_TOKEN_KEY);
  printf("WSI_TOKEN_VERSION = 0o%o\n", WSI_TOKEN_VERSION);
  printf("WSI_TOKEN_SWORIGIN = 0o%o\n", WSI_TOKEN_SWORIGIN);
  printf("WSI_TOKEN_HTTP_COLON_AUTHORITY = 0o%o\n", WSI_TOKEN_HTTP_COLON_AUTHORITY);
  printf("WSI_TOKEN_HTTP_COLON_METHOD = 0o%o\n", WSI_TOKEN_HTTP_COLON_METHOD);
  printf("WSI_TOKEN_HTTP_COLON_PATH = 0o%o\n", WSI_TOKEN_HTTP_COLON_PATH);
  printf("WSI_TOKEN_HTTP_COLON_SCHEME = 0o%o\n", WSI_TOKEN_HTTP_COLON_SCHEME);
  printf("WSI_TOKEN_HTTP_COLON_STATUS = 0o%o\n", WSI_TOKEN_HTTP_COLON_STATUS);
  printf("WSI_TOKEN_HTTP_ACCEPT_CHARSET = 0o%o\n", WSI_TOKEN_HTTP_ACCEPT_CHARSET);
  printf("WSI_TOKEN_HTTP_ACCEPT_RANGES = 0o%o\n", WSI_TOKEN_HTTP_ACCEPT_RANGES);
  printf("WSI_TOKEN_HTTP_ACCESS_CONTROL_ALLOW_ORIGIN = 0o%o\n",
         WSI_TOKEN_HTTP_ACCESS_CONTROL_ALLOW_ORIGIN);
  printf("WSI_TOKEN_HTTP_AGE = 0o%o\n", WSI_TOKEN_HTTP_AGE);
  printf("WSI_TOKEN_HTTP_ALLOW = 0o%o\n", WSI_TOKEN_HTTP_ALLOW);
  printf("WSI_TOKEN_HTTP_CONTENT_DISPOSITION = 0o%o\n", WSI_TOKEN_HTTP_CONTENT_DISPOSITION);
  printf("WSI_TOKEN_HTTP_CONTENT_ENCODING = 0o%o\n", WSI_TOKEN_HTTP_CONTENT_ENCODING);
  printf("WSI_TOKEN_HTTP_CONTENT_LANGUAGE = 0o%o\n", WSI_TOKEN_HTTP_CONTENT_LANGUAGE);
  printf("WSI_TOKEN_HTTP_CONTENT_LOCATION = 0o%o\n", WSI_TOKEN_HTTP_CONTENT_LOCATION);
  printf("WSI_TOKEN_HTTP_CONTENT_RANGE = 0o%o\n", WSI_TOKEN_HTTP_CONTENT_RANGE);
  printf("WSI_TOKEN_HTTP_ETAG = 0o%o\n", WSI_TOKEN_HTTP_ETAG);
  printf("WSI_TOKEN_HTTP_EXPECT = 0o%o\n", WSI_TOKEN_HTTP_EXPECT);
  printf("WSI_TOKEN_HTTP_EXPIRES = 0o%o\n", WSI_TOKEN_HTTP_EXPIRES);
  printf("WSI_TOKEN_HTTP_FROM = 0o%o\n", WSI_TOKEN_HTTP_FROM);
  printf("WSI_TOKEN_HTTP_IF_MATCH = 0o%o\n", WSI_TOKEN_HTTP_IF_MATCH);
  printf("WSI_TOKEN_HTTP_IF_RANGE = 0o%o\n", WSI_TOKEN_HTTP_IF_RANGE);
  printf("WSI_TOKEN_HTTP_IF_UNMODIFIED_SINCE = 0o%o\n", WSI_TOKEN_HTTP_IF_UNMODIFIED_SINCE);
  printf("WSI_TOKEN_HTTP_LAST_MODIFIED = 0o%o\n", WSI_TOKEN_HTTP_LAST_MODIFIED);
  printf("WSI_TOKEN_HTTP_LINK = 0o%o\n", WSI_TOKEN_HTTP_LINK);
  printf("WSI_TOKEN_HTTP_LOCATION = 0o%o\n", WSI_TOKEN_HTTP_LOCATION);
  printf("WSI_TOKEN_HTTP_MAX_FORWARDS = 0o%o\n", WSI_TOKEN_HTTP_MAX_FORWARDS);
  printf("WSI_TOKEN_HTTP_PROXY_AUTHENTICATE = 0o%o\n", WSI_TOKEN_HTTP_PROXY_AUTHENTICATE);
  printf("WSI_TOKEN_HTTP_PROXY_AUTHORIZATION = 0o%o\n", WSI_TOKEN_HTTP_PROXY_AUTHORIZATION);
  printf("WSI_TOKEN_HTTP_REFRESH = 0o%o\n", WSI_TOKEN_HTTP_REFRESH);
  printf("WSI_TOKEN_HTTP_RETRY_AFTER = 0o%o\n", WSI_TOKEN_HTTP_RETRY_AFTER);
  printf("WSI_TOKEN_HTTP_SERVER = 0o%o\n", WSI_TOKEN_HTTP_SERVER);
  printf("WSI_TOKEN_HTTP_SET_COOKIE = 0o%o\n", WSI_TOKEN_HTTP_SET_COOKIE);
  printf("WSI_TOKEN_HTTP_STRICT_TRANSPORT_SECURITY = 0o%o\n",
         WSI_TOKEN_HTTP_STRICT_TRANSPORT_SECURITY);
  printf("WSI_TOKEN_HTTP_TRANSFER_ENCODING = 0o%o\n", WSI_TOKEN_HTTP_TRANSFER_ENCODING);
  printf("WSI_TOKEN_HTTP_USER_AGENT = 0o%o\n", WSI_TOKEN_HTTP_USER_AGENT);
  printf("WSI_TOKEN_HTTP_VARY = 0o%o\n", WSI_TOKEN_HTTP_VARY);
  printf("WSI_TOKEN_HTTP_VIA = 0o%o\n", WSI_TOKEN_HTTP_VIA);
  printf("WSI_TOKEN_HTTP_WWW_AUTHENTICATE = 0o%o\n", WSI_TOKEN_HTTP_WWW_AUTHENTICATE);
  printf("WSI_TOKEN_PATCH_URI = 0o%o\n", WSI_TOKEN_PATCH_URI);
  printf("WSI_TOKEN_PUT_URI = 0o%o\n", WSI_TOKEN_PUT_URI);
  printf("WSI_TOKEN_DELETE_URI = 0o%o\n", WSI_TOKEN_DELETE_URI);
  printf("WSI_TOKEN_HTTP_URI_ARGS = 0o%o\n", WSI_TOKEN_HTTP_URI_ARGS);
  printf("WSI_TOKEN_PROXY = 0o%o\n", WSI_TOKEN_PROXY);
  printf("WSI_TOKEN_HTTP_X_REAL_IP = 0o%o\n", WSI_TOKEN_HTTP_X_REAL_IP);
  printf("WSI_TOKEN_HTTP1_0 = 0o%o\n", WSI_TOKEN_HTTP1_0);
  printf("WSI_TOKEN_X_FORWARDED_FOR = 0o%o\n", WSI_TOKEN_X_FORWARDED_FOR);
  printf("WSI_TOKEN_CONNECT = 0o%o\n", WSI_TOKEN_CONNECT);
  printf("WSI_TOKEN_HEAD_URI = 0o%o\n", WSI_TOKEN_HEAD_URI);
  printf("WSI_TOKEN_TE = 0o%o\n", WSI_TOKEN_TE);
  printf("WSI_TOKEN_REPLAY_NONCE = 0o%o\n", WSI_TOKEN_REPLAY_NONCE);
  printf("WSI_TOKEN_COLON_PROTOCOL = 0o%o\n", WSI_TOKEN_COLON_PROTOCOL);
  printf("WSI_TOKEN_X_AUTH_TOKEN = 0o%o\n", WSI_TOKEN_X_AUTH_TOKEN);
  printf("WSI_TOKEN_DSS_SIGNATURE = 0o%o\n", WSI_TOKEN_DSS_SIGNATURE);
  printf("WSI_TOKEN_COUNT = 0o%o\n", WSI_TOKEN_COUNT);
  printf("WSI_TOKEN_NAME_PART = 0o%o\n", WSI_TOKEN_NAME_PART);
  printf("WSI_TOKEN_UNKNOWN_VALUE_PART = 0o%o\n", WSI_TOKEN_UNKNOWN_VALUE_PART);
  printf("WSI_TOKEN_SKIPPING = 0o%o\n", WSI_TOKEN_SKIPPING);
  printf("WSI_TOKEN_SKIPPING_SAW_CR = 0o%o\n", WSI_TOKEN_SKIPPING_SAW_CR);
  printf("WSI_TOKEN_COUNT = 0o%o\n", WSI_TOKEN_COUNT);
  printf("WSI_TOKEN_HTTP_URI_ARGS = 0o%o\n", WSI_TOKEN_HTTP_URI_ARGS);
  printf("--fcntl flags\n");

  printf("FD_CLOEXEC: 0o%o\n", FD_CLOEXEC);
  printf("F_DUPFD: 0o%o\n", F_DUPFD);
  printf("F_DUPFD_CLOEXEC: 0o%o\n", F_DUPFD_CLOEXEC);
  printf("F_GETFD: 0o%o\n", F_GETFD);
  printf("F_GETFL: 0o%o\n", F_GETFL);
  printf("F_GETLK: 0o%o\n", F_GETLK);
  printf("F_RDLCK: 0o%o\n", F_RDLCK);
  printf("F_SETFD: 0o%o\n", F_SETFD);
  printf("F_SETFL: 0o%o\n", F_SETFL);
  printf("F_SETLK: 0o%o\n", F_SETLK);
  printf("F_SETLKW: 0o%o\n", F_SETLKW);
  printf("F_UNLCK: 0o%o\n", F_UNLCK);
  printf("F_WRLCK: 0o%o\n", F_WRLCK);
  printf("O_APPEND: 0o%o\n", O_APPEND);
  printf("O_ASYNC: 0o%o\n", O_ASYNC);
  printf("O_CLOEXEC: 0o%o\n", O_CLOEXEC);
  printf("O_CREAT: 0o%o\n", O_CREAT);
  printf("O_DSYNC: 0o%o\n", O_DSYNC);
  printf("O_EXCL: 0o%o\n", O_EXCL);
  printf("O_NOCTTY: 0o%o\n", O_NOCTTY);
  printf("O_NONBLOCK: 0o%o\n", O_NONBLOCK);
  printf("O_RDONLY: 0o%o\n", O_RDONLY);
  printf("O_RDWR: 0o%o\n", O_RDWR);
  printf("O_SYNC: 0o%o\n", O_SYNC);
  printf("O_TRUNC: 0o%o\n", O_TRUNC);
  printf("O_WRONLY: 0o%o\n", O_WRONLY);
  printf("EAGAIN: %i\n", EAGAIN);
  printf("EWOULDBLOCK: %i\n", EWOULDBLOCK);
  printf("SIG_IGN: %i\n", SIG_IGN);
  printf("SIGINT: %i\n", SIGINT);
  printf("%s = %d\n", "FNM_NOESCAPE", FNM_NOESCAPE);
  printf("%s = %d\n", "FNM_PATHNAME", FNM_PATHNAME);
  printf("%s = %d\n", "FNM_PERIOD", FNM_PERIOD);
  printf("%s = %d\n", "FNM_EXTMATCH", FNM_EXTMATCH);
  printf("%s = %d\n", "FNM_NOMATCH", FNM_NOMATCH);
  printf("%s = %d\n", "FNM_FILE_NAME", FNM_FILE_NAME);
  printf("%s = %d\n", "FNM_LEADING_DIR", FNM_LEADING_DIR);
  printf("%s = %d\n", "FNM_CASEFOLD", FNM_CASEFOLD);

  printf("MYSQL_TYPE_DECIMAL = %d\n", MYSQL_TYPE_DECIMAL);
  printf("MYSQL_TYPE_TINY = %d\n", MYSQL_TYPE_TINY);
  printf("MYSQL_TYPE_SHORT = %d\n", MYSQL_TYPE_SHORT);
  printf("MYSQL_TYPE_LONG = %d\n", MYSQL_TYPE_LONG);
  printf("MYSQL_TYPE_FLOAT = %d\n", MYSQL_TYPE_FLOAT);
  printf("MYSQL_TYPE_DOUBLE = %d\n", MYSQL_TYPE_DOUBLE);
  printf("MYSQL_TYPE_NULL = %d\n", MYSQL_TYPE_NULL);
  printf("MYSQL_TYPE_TIMESTAMP = %d\n", MYSQL_TYPE_TIMESTAMP);
  printf("MYSQL_TYPE_LONGLONG = %d\n", MYSQL_TYPE_LONGLONG);
  printf("MYSQL_TYPE_INT24 = %d\n", MYSQL_TYPE_INT24);
  printf("MYSQL_TYPE_DATE = %d\n", MYSQL_TYPE_DATE);
  printf("MYSQL_TYPE_TIME = %d\n", MYSQL_TYPE_TIME);
  printf("MYSQL_TYPE_DATETIME = %d\n", MYSQL_TYPE_DATETIME);
  printf("MYSQL_TYPE_YEAR = %d\n", MYSQL_TYPE_YEAR);
  printf("MYSQL_TYPE_NEWDATE = %d\n", MYSQL_TYPE_NEWDATE);
  printf("MYSQL_TYPE_VARCHAR = %d\n", MYSQL_TYPE_VARCHAR);
  printf("MYSQL_TYPE_BIT = %d\n", MYSQL_TYPE_BIT);
  printf("MYSQL_TYPE_TIMESTAMP2 = %d\n", MYSQL_TYPE_TIMESTAMP2);
  printf("MYSQL_TYPE_DATETIME2 = %d\n", MYSQL_TYPE_DATETIME2);
  printf("MYSQL_TYPE_TIME2 = %d\n", MYSQL_TYPE_TIME2);
  // printf("MYSQL_TYPE_BLOB_COMPRESSED = %d\n", MYSQL_TYPE_BLOB_COMPRESSED);
  // printf("MYSQL_TYPE_VARCHAR_COMPRESSED = %d\n", MYSQL_TYPE_VARCHAR_COMPRESSED);
  printf("MYSQL_TYPE_NEWDECIMAL = %d\n", MYSQL_TYPE_NEWDECIMAL);
  printf("MYSQL_TYPE_ENUM = %d\n", MYSQL_TYPE_ENUM);
  printf("MYSQL_TYPE_SET = %d\n", MYSQL_TYPE_SET);
  printf("MYSQL_TYPE_TINY_BLOB = %d\n", MYSQL_TYPE_TINY_BLOB);
  printf("MYSQL_TYPE_MEDIUM_BLOB = %d\n", MYSQL_TYPE_MEDIUM_BLOB);
  printf("MYSQL_TYPE_LONG_BLOB = %d\n", MYSQL_TYPE_LONG_BLOB);
  printf("MYSQL_TYPE_BLOB = %d\n", MYSQL_TYPE_BLOB);
  printf("MYSQL_TYPE_VAR_STRING = %d\n", MYSQL_TYPE_VAR_STRING);
  printf("MYSQL_TYPE_STRING = %d\n", MYSQL_TYPE_STRING);
  printf("MYSQL_TYPE_GEOMETRY = %d\n", MYSQL_TYPE_GEOMETRY);
  printf("%s",
         "import REPL from 'repl';\n"
         "import fs from 'fs';\n"
         "const history = '%s/.%s_history';\n"
         "globalThis.repl = new REPL((__filename ?? '%s').replace(/.*\\//g, "
         "'').replace(/\\.js$/g, ''), false);\n"
         "repl.loadSaveOptions();\n"
         "repl.historyLoad(null, fs);\n"
         "repl.directives = { i: [\n"
         "  (name => import(name).then(m => {\n"
         "    let id = name.slice(name.lastIndexOf('/') + 1).replace(/\\.[^\\/.]+$/g, '');\n"
         "    globalThis[id] = m;\n"
         "  }).catch(() => repl.printStatus(`ERROR: module '${name}' not found\\n`))),\n"
         " 'import a module'\n"
         "] };\n"
         "repl.show = console.log;\n"
         "repl.runSync();\n");

  return 0;
}

#include <stdio.h>
#include <errno.h>

int
main() {
  printf("E2BIG = %i\n", E2BIG);
  printf("EACCES = %i\n", EACCES);
  printf("EADDRINUSE = %i\n", EADDRINUSE);
  printf("EADDRNOTAVAIL = %i\n", EADDRNOTAVAIL);
  printf("EADV = %i\n", EADV);
  printf("EAFNOSUPPORT = %i\n", EAFNOSUPPORT);
  printf("EAGAIN = %i\n", EAGAIN);
  printf("EALREADY = %i\n", EALREADY);
  printf("EBADE = %i\n", EBADE);
  printf("EBADF = %i\n", EBADF);
  printf("EBADFD = %i\n", EBADFD);
  printf("EBADMSG = %i\n", EBADMSG);
  printf("EBADR = %i\n", EBADR);
  printf("EBADRQC = %i\n", EBADRQC);
  printf("EBADSLT = %i\n", EBADSLT);
  printf("EBFONT = %i\n", EBFONT);
  printf("EBUSY = %i\n", EBUSY);
  printf("ECANCELED = %i\n", ECANCELED);
  printf("ECHILD = %i\n", ECHILD);
  printf("ECHRNG = %i\n", ECHRNG);
  printf("ECOMM = %i\n", ECOMM);
  printf("ECONNABORTED = %i\n", ECONNABORTED);
  printf("ECONNREFUSED = %i\n", ECONNREFUSED);
  printf("ECONNRESET = %i\n", ECONNRESET);
  printf("EDEADLK = %i\n", EDEADLK);
  printf("EDEADLOCK = %i\n", EDEADLOCK);
  printf("EDESTADDRREQ = %i\n", EDESTADDRREQ);
  printf("EDOM = %i\n", EDOM);
  printf("EDOTDOT = %i\n", EDOTDOT);
  printf("EDQUOT = %i\n", EDQUOT);
  printf("EEXIST = %i\n", EEXIST);
  printf("EFAULT = %i\n", EFAULT);
  printf("EFBIG = %i\n", EFBIG);
  printf("EHOSTDOWN = %i\n", EHOSTDOWN);
  printf("EHOSTUNREACH = %i\n", EHOSTUNREACH);
  printf("EIDRM = %i\n", EIDRM);
  printf("EILSEQ = %i\n", EILSEQ);
  printf("EINPROGRESS = %i\n", EINPROGRESS);
  printf("EINTR = %i\n", EINTR);
  printf("EINVAL = %i\n", EINVAL);
  printf("EIO = %i\n", EIO);
  printf("EISCONN = %i\n", EISCONN);
  printf("EISDIR = %i\n", EISDIR);
  printf("EISNAM = %i\n", EISNAM);
  printf("EL2HLT = %i\n", EL2HLT);
  printf("EL2NSYNC = %i\n", EL2NSYNC);
  printf("EL3HLT = %i\n", EL3HLT);
  printf("EL3RST = %i\n", EL3RST);
  printf("ELIBACC = %i\n", ELIBACC);
  printf("ELIBBAD = %i\n", ELIBBAD);
  printf("ELIBEXEC = %i\n", ELIBEXEC);
  printf("ELIBMAX = %i\n", ELIBMAX);
  printf("ELIBSCN = %i\n", ELIBSCN);
  printf("ELNRNG = %i\n", ELNRNG);
  printf("ELOOP = %i\n", ELOOP);
  printf("EMEDIUMTYPE = %i\n", EMEDIUMTYPE);
  printf("EMFILE = %i\n", EMFILE);
  printf("EMLINK = %i\n", EMLINK);
  printf("EMSGSIZE = %i\n", EMSGSIZE);
  printf("EMULTIHOP = %i\n", EMULTIHOP);
  printf("ENAMETOOLONG = %i\n", ENAMETOOLONG);
  printf("ENAVAIL = %i\n", ENAVAIL);
  printf("ENETDOWN = %i\n", ENETDOWN);
  printf("ENETRESET = %i\n", ENETRESET);
  printf("ENETUNREACH = %i\n", ENETUNREACH);
  printf("ENFILE = %i\n", ENFILE);
  printf("ENOANO = %i\n", ENOANO);
  printf("ENOBUFS = %i\n", ENOBUFS);
  printf("ENOCSI = %i\n", ENOCSI);
  printf("ENODATA = %i\n", ENODATA);
  printf("ENODEV = %i\n", ENODEV);
  printf("ENOENT = %i\n", ENOENT);
  printf("ENOEXEC = %i\n", ENOEXEC);
  printf("ENOLCK = %i\n", ENOLCK);
  printf("ENOLINK = %i\n", ENOLINK);
  printf("ENOMEDIUM = %i\n", ENOMEDIUM);
  printf("ENOMEM = %i\n", ENOMEM);
  printf("ENOMSG = %i\n", ENOMSG);
  printf("ENONET = %i\n", ENONET);
  printf("ENOPKG = %i\n", ENOPKG);
  printf("ENOPROTOOPT = %i\n", ENOPROTOOPT);
  printf("ENOSPC = %i\n", ENOSPC);
  printf("ENOSR = %i\n", ENOSR);
  printf("ENOSTR = %i\n", ENOSTR);
  printf("ENOSYS = %i\n", ENOSYS);
  printf("ENOTBLK = %i\n", ENOTBLK);
  printf("ENOTCONN = %i\n", ENOTCONN);
  printf("ENOTDIR = %i\n", ENOTDIR);
  printf("ENOTEMPTY = %i\n", ENOTEMPTY);
  printf("ENOTNAM = %i\n", ENOTNAM);
  printf("ENOTSOCK = %i\n", ENOTSOCK);
  printf("ENOTSUP = %i\n", ENOTSUP);
  printf("ENOTTY = %i\n", ENOTTY);
  printf("ENOTUNIQ = %i\n", ENOTUNIQ);
  printf("ENXIO = %i\n", ENXIO);
  printf("EOPNOTSUPP = %i\n", EOPNOTSUPP);
  printf("EOVERFLOW = %i\n", EOVERFLOW);
  printf("EPERM = %i\n", EPERM);
  printf("EPFNOSUPPORT = %i\n", EPFNOSUPPORT);
  printf("EPIPE = %i\n", EPIPE);
  printf("EPROTO = %i\n", EPROTO);
  printf("EPROTONOSUPPORT = %i\n", EPROTONOSUPPORT);
  printf("EPROTOTYPE = %i\n", EPROTOTYPE);
  printf("ERANGE = %i\n", ERANGE);
  printf("EREMCHG = %i\n", EREMCHG);
  printf("EREMOTE = %i\n", EREMOTE);
  printf("EREMOTEIO = %i\n", EREMOTEIO);
  printf("ERESTART = %i\n", ERESTART);
  printf("EROFS = %i\n", EROFS);
  printf("ESHUTDOWN = %i\n", ESHUTDOWN);
  printf("ESOCKTNOSUPPORT = %i\n", ESOCKTNOSUPPORT);
  printf("ESPIPE = %i\n", ESPIPE);
  printf("ESRCH = %i\n", ESRCH);
  printf("ESRMNT = %i\n", ESRMNT);
  printf("ESTALE = %i\n", ESTALE);
  printf("ESTRPIPE = %i\n", ESTRPIPE);
  printf("ETIME = %i\n", ETIME);
  printf("ETIMEDOUT = %i\n", ETIMEDOUT);
  printf("ETOOMANYREFS = %i\n", ETOOMANYREFS);
  printf("ETXTBSY = %i\n", ETXTBSY);
  printf("EUCLEAN = %i\n", EUCLEAN);
  printf("EUNATCH = %i\n", EUNATCH);
  printf("EUSERS = %i\n", EUSERS);
  printf("EWOULDBLOCK = %i\n", EWOULDBLOCK);
  printf("EXDEV = %i\n", EXDEV);
  printf("EXFULL = %i\n", EXFULL);

  return 0;
}

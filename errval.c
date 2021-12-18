#include <stdio.h> 
#include <errno.h> 
 
int
main() {
  
  printf("EPERM = %i\n", EPERM);
  printf("ENOENT = %i\n", ENOENT);
  printf("ESRCH = %i\n", ESRCH);
  printf("EINTR = %i\n", EINTR);
  printf("EIO = %i\n", EIO);
  printf("ENXIO = %i\n", ENXIO);
  printf("E2BIG = %i\n", E2BIG);
  printf("ENOEXEC = %i\n", ENOEXEC);
  printf("EBADF = %i\n", EBADF);
  printf("ECHILD = %i\n", ECHILD);
  printf("EAGAIN = %i\n", EAGAIN);
  printf("ENOMEM = %i\n", ENOMEM);
  printf("EACCES = %i\n", EACCES);
  printf("EFAULT = %i\n", EFAULT);
  printf("EBUSY = %i\n", EBUSY);
  printf("EEXIST = %i\n", EEXIST);
  printf("EXDEV = %i\n", EXDEV);
  printf("ENODEV = %i\n", ENODEV);
  printf("ENOTDIR = %i\n", ENOTDIR);
  printf("EISDIR = %i\n", EISDIR);
  printf("EINVAL = %i\n", EINVAL);
  printf("ENFILE = %i\n", ENFILE);
  printf("EMFILE = %i\n", EMFILE);
  printf("ENOTTY = %i\n", ENOTTY);
  printf("ETXTBSY = %i\n", ETXTBSY);
  printf("EFBIG = %i\n", EFBIG);
  printf("ENOSPC = %i\n", ENOSPC);
  printf("ESPIPE = %i\n", ESPIPE);
  printf("EROFS = %i\n", EROFS);
  printf("EMLINK = %i\n", EMLINK);
  printf("EPIPE = %i\n", EPIPE);
  printf("EDOM = %i\n", EDOM);
  printf("ERANGE = %i\n", ERANGE);
  printf("EDEADLK = %i\n", EDEADLK);
  printf("ENAMETOOLONG = %i\n", ENAMETOOLONG);
  printf("ENOLCK = %i\n", ENOLCK);
  printf("ENOSYS = %i\n", ENOSYS);
  printf("ENOTEMPTY = %i\n", ENOTEMPTY);
  printf("ENOMSG = %i\n", ENOMSG);
  printf("EIDRM = %i\n", EIDRM);
  printf("ENOLINK = %i\n", ENOLINK);
  printf("EPROTO = %i\n", EPROTO);
  printf("EBADMSG = %i\n", EBADMSG);
  printf("EOVERFLOW = %i\n", EOVERFLOW);
  printf("EILSEQ = %i\n", EILSEQ);
  printf("ENOTSOCK = %i\n", ENOTSOCK);
  printf("EDESTADDRREQ = %i\n", EDESTADDRREQ);
  printf("EMSGSIZE = %i\n", EMSGSIZE);
  printf("EPROTOTYPE = %i\n", EPROTOTYPE);
  printf("ENOPROTOOPT = %i\n", ENOPROTOOPT);
  printf("EPROTONOSUPPORT = %i\n", EPROTONOSUPPORT);
  printf("EOPNOTSUPP = %i\n", EOPNOTSUPP);
  printf("EAFNOSUPPORT = %i\n", EAFNOSUPPORT);
  printf("EADDRINUSE = %i\n", EADDRINUSE);
  printf("EADDRNOTAVAIL = %i\n", EADDRNOTAVAIL);
  printf("ENETDOWN = %i\n", ENETDOWN);
  printf("ENETUNREACH = %i\n", ENETUNREACH);
  printf("ENETRESET = %i\n", ENETRESET);
  printf("ECONNABORTED = %i\n", ECONNABORTED);
  printf("ECONNRESET = %i\n", ECONNRESET);
  printf("ENOBUFS = %i\n", ENOBUFS);
  printf("EISCONN = %i\n", EISCONN);
  printf("ENOTCONN = %i\n", ENOTCONN);
  printf("ETIMEDOUT = %i\n", ETIMEDOUT);
  printf("ECONNREFUSED = %i\n", ECONNREFUSED);
  printf("EHOSTUNREACH = %i\n", EHOSTUNREACH);
  printf("EALREADY = %i\n", EALREADY);
  printf("EINPROGRESS = %i\n", EINPROGRESS);
  printf("ECANCELED = %i\n", ECANCELED);
  printf("EOWNERDEAD = %i\n", EOWNERDEAD);
  printf("ENOTRECOVERABLE = %i\n", ENOTRECOVERABLE);
  printf("EWOULDBLOCK = %i\n", EWOULDBLOCK);

  return 0;
}

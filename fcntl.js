export const F_DUPFD = 0;
export const F_GETFD = 1;
export const F_SETFD = 2;
export const F_GETFL = 3;
export const F_SETFL = 4;
export const F_GETLK = 5;
export const F_SETLK = 6;
export const F_SETLKW = 7;
export const F_GETLK64 = 8;
export const F_SETLK64 = 9;
export const F_SETLKW64 = 10;
export const F_GETOWN = 11;
export const F_SETOWN = 12;
export const F_SETSIG = 13;
export const F_GETSIG = 14;
export const O_NONBLOCK = 0x0800;
export const O_LARGEFILE = 0x8000;
export const O_NOFOLLOW = 0x00020000;
export const O_CLOEXEC = 0x00080000;
export const O_NOATIME = 0x00040000;

export function fcntl(fd, cmd, arg) {
  return import('ffi')
    .then(({ define, dlsym, call }) => {
      let fp = dlsym(RTLD_DEFAULT, 'fcntl');
      define('fcntl', fp, null, 'int', 'int', 'int', 'int');

      return call('fcntl', fd, cmd, arg);
    })
    .catch(err => -1);
}

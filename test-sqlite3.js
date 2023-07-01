import { SQLITE_OK, SQLITE_ERROR, SQLITE_INTERNAL, SQLITE_PERM, SQLITE_ABORT, SQLITE_BUSY, SQLITE_LOCKED, SQLITE_NOMEM, SQLITE_READONLY, SQLITE_INTERRUPT, SQLITE_IOERR, SQLITE_CORRUPT, SQLITE_NOTFOUND, SQLITE_FULL, SQLITE_CANTOPEN, SQLITE_PROTOCOL, SQLITE_EMPTY, SQLITE_SCHEMA, SQLITE_TOOBIG, SQLITE_CONSTRAINT, SQLITE_MISMATCH, SQLITE_MISUSE, SQLITE_NOLFS, SQLITE_AUTH, SQLITE_FORMAT, SQLITE_RANGE, SQLITE_NOTADB, SQLITE_NOTICE, SQLITE_WARNING, SQLITE_ROW, SQLITE_DONE, sqlite3_close, sqlite3_exec, sqlite3_free, sqlite3_open, sqlite3_errmsg } from './sqlite3.js';

function main(...args) {
  let ret,
    a = new Uint32Array(2);

  ret = sqlite3_open('/home/roman/.config/google-chrome/Default/History', a.buffer);

  console.log('ret', ret);
}

main(...scriptArgs.slice(1));

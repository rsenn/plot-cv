import { call, define, dlopen, dlsym, RTLD_NOW } from 'ffi';

const libsqlite3 = dlopen('libsqlite3.so.0', RTLD_NOW);

export const SQLITE_OK = 0; /* Successful result */
export const SQLITE_ERROR = 1; /* Generic error */
export const SQLITE_INTERNAL = 2; /* Internal logic error in SQLite */
export const SQLITE_PERM = 3; /* Access permission denied */
export const SQLITE_ABORT = 4; /* Callback routine requested an abort */
export const SQLITE_BUSY = 5; /* The database file is locked */
export const SQLITE_LOCKED = 6; /* A table in the database is locked */
export const SQLITE_NOMEM = 7; /* A malloc() failed */
export const SQLITE_READONLY = 8; /* Attempt to write a readonly database */
export const SQLITE_INTERRUPT = 9; /* Operation terminated by sqlite3_interrupt()*/
export const SQLITE_IOERR = 10; /* Some kind of disk I/O error occurred */
export const SQLITE_CORRUPT = 11; /* The database disk image is malformed */
export const SQLITE_NOTFOUND = 12; /* Unknown opcode in sqlite3_file_control() */
export const SQLITE_FULL = 13; /* Insertion failed because database is full */
export const SQLITE_CANTOPEN = 14; /* Unable to open the database file */
export const SQLITE_PROTOCOL = 15; /* Database lock protocol error */
export const SQLITE_EMPTY = 16; /* Internal use only */
export const SQLITE_SCHEMA = 17; /* The database schema changed */
export const SQLITE_TOOBIG = 18; /* String or BLOB exceeds size limit */
export const SQLITE_CONSTRAINT = 19; /* Abort due to constraint violation */
export const SQLITE_MISMATCH = 20; /* Data type mismatch */
export const SQLITE_MISUSE = 21; /* Library used incorrectly */
export const SQLITE_NOLFS = 22; /* Uses OS features not supported on host */
export const SQLITE_AUTH = 23; /* Authorization denied */
export const SQLITE_FORMAT = 24; /* Not used */
export const SQLITE_RANGE = 25; /* 2nd parameter to sqlite3_bind out of range */
export const SQLITE_NOTADB = 26; /* File opened that is not a database file */
export const SQLITE_NOTICE = 27; /* Notifications from sqlite3_log() */
export const SQLITE_WARNING = 28; /* Warnings from sqlite3_log() */
export const SQLITE_ROW = 100; /* sqlite3_step() has another row ready */
export const SQLITE_DONE = 101; /* sqlite3_step() has finished executing */

/**
 * @function sqlite3_close
 *
 * @param    {Number}        arg1
 *
 * @return   {Number}
 */
define('sqlite3_close', dlsym(libsqlite3, 'sqlite3_close'), null, 'int', 'void *');

export function sqlite3_close(arg1) {
  return call('sqlite3_close', arg1);
}

/**
 * @function sqlite3_exec
 *
 * @param    {Number}        arg1
 * @param    {String}        sql
 * @param    {Number}        callback
 * @param    {Number}        arg4
 * @param    {Number}        errmsg
 *
 * @return   {Number}
 */
define('sqlite3_exec', dlsym(libsqlite3, 'sqlite3_exec'), null, 'int', 'void *', 'char *', 'long', 'void *', 'void *');

export function sqlite3_exec(arg1, sql, callback, arg4, errmsg) {
  return call('sqlite3_exec', arg1, sql, callback, arg4, errmsg);
}

/**
 * @function sqlite3_free
 *
 * @param    {Number}        arg1
 */
define('sqlite3_free', dlsym(libsqlite3, 'sqlite3_free'), null, 'void', 'void *');
export function sqlite3_free(arg1) {
  call('sqlite3_free', arg1);
}

/**
 * @function sqlite3_open
 *
 * @param    {String}        filename
 * @param    {Number}        ppDb
 *
 * @return   {Number}
 */
define('sqlite3_open', dlsym(libsqlite3, 'sqlite3_open'), null, 'int', 'char *', 'void *');
export function sqlite3_open(filename, ppDb) {
  return call('sqlite3_open', filename, ppDb);
}

/**
 * @function sqlite3_errmsg
 *
 * @param    {Number}        arg1
 *
 * @return   {String}
 */
define('sqlite3_errmsg', dlsym(libsqlite3, 'sqlite3_errmsg'), null, 'char *', 'void *');
export function sqlite3_errmsg(arg1) {
  return call('sqlite3_errmsg', arg1);
}
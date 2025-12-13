import { call, define, dlopen, dlsym, RTLD_NOW } from 'ffi';

const libarchive = dlopen('libarchive.so.13', RTLD_NOW);

/**
 * @function archive_version_number
 *
 * @return   {Number}
 */
define('archive_version_number', dlsym(libarchive, 'archive_version_number'), null, 'int');

export function archive_version_number() {
  return call('archive_version_number');
}

/**
 * @function archive_version_string
 *
 * @return   {String}
 */
define('archive_version_string', dlsym(libarchive, 'archive_version_string'), null, 'char *');

export function archive_version_string() {
  return call('archive_version_string');
}

/**
 * @function archive_version_details
 *
 * @return   {String}
 */
define('archive_version_details', dlsym(libarchive, 'archive_version_details'), null, 'char *');

export function archive_version_details() {
  return call('archive_version_details');
}

/**
 * @function archive_zlib_version
 *
 * @return   {String}
 */
define('archive_zlib_version', dlsym(libarchive, 'archive_zlib_version'), null, 'char *');

export function archive_zlib_version() {
  return call('archive_zlib_version');
}

/**
 * @function archive_liblzma_version
 *
 * @return   {String}
 */
define('archive_liblzma_version', dlsym(libarchive, 'archive_liblzma_version'), null, 'char *');

export function archive_liblzma_version() {
  return call('archive_liblzma_version');
}

/**
 * @function archive_bzlib_version
 *
 * @return   {String}
 */
define('archive_bzlib_version', dlsym(libarchive, 'archive_bzlib_version'), null, 'char *');
export function archive_bzlib_version() {
  return call('archive_bzlib_version');
}

/**
 * @function archive_liblz4_version
 *
 * @return   {String}
 */
define('archive_liblz4_version', dlsym(libarchive, 'archive_liblz4_version'), null, 'char *');
export function archive_liblz4_version() {
  return call('archive_liblz4_version');
}

/**
 * @function archive_libzstd_version
 *
 * @return   {String}
 */
define('archive_libzstd_version', dlsym(libarchive, 'archive_libzstd_version'), null, 'char *');
export function archive_libzstd_version() {
  return call('archive_libzstd_version');
}

/**
 * @function archive_read_new
 *
 * @return   {Number}
 */
define('archive_read_new', dlsym(libarchive, 'archive_read_new'), null, 'void *');
export function archive_read_new() {
  return call('archive_read_new');
}

/**
 * @function archive_read_support_compression_all
 *
 * @param    {Number}        arg1
 *
 * @return   {Number}
 */
define('archive_read_support_compression_all', dlsym(libarchive, 'archive_read_support_compression_all'), null, 'int', 'void *');
export function archive_read_support_compression_all(arg1) {
  return call('archive_read_support_compression_all', arg1);
}
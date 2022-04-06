/* CONFIG_VERSION - QuickJS version */
#define CONFIG_VERSION "2021-03-27"

/* CONFIG_SHEXT - Shared module extension */
#define CONFIG_SHEXT ".so"

/* CONFIG_ALL_UNICODE - Define this if you want all-Unicode support */
/* #undef CONFIG_ALL_UNICODE */

/* CONFIG_ATOMICS - Define this if you want 'Atomics' object [https://tc39.es/ecma262/#sec-atomics-object] */
/* #undef CONFIG_ATOMICS */

/* CONFIG_BIGNUM - Define this if you want BigNum support */
#define CONFIG_BIGNUM 1

/* CONFIG_PRINTF_RNDN - define it if printf uses the RNDN rounding mode instead of RNDNA */
/* #undef CONFIG_PRINTF_RNDN */

/* CONFIG_CHECK_JSVALUE - define it if printf uses the RNDN rounding mode instead of RNDNA */
/* #undef CONFIG_PRINTF_RNDN */

/* CONFIG_DEBUGGER - define it if you want debug protocol support */
#define CONFIG_DEBUGGER 1

/* QUICKJS_C_MODULE_DIR - Native C module directory */
#define QUICKJS_C_MODULE_DIR "/usr/local/lib/x86_64-linux-gnu/quickjs"

/* QUICKJS_JS_MODULE_DIR - JS module directory */
#define QUICKJS_JS_MODULE_DIR "/usr/local/lib/quickjs"

/* QUICKJS_MODULE_PATH - Module search path */
#define QUICKJS_MODULE_PATH "/usr/local/lib/x86_64-linux-gnu/quickjs;/usr/local/lib/quickjs"

/* HAVE_MALLOC_USABLE_SIZE - Define this if you have a working malloc_usable_size() function */
#define HAVE_MALLOC_USABLE_SIZE 1

/* PROFILE - Define this if you want to profile the code */
/* #undef PROFILE */

/* USE_WORKER - Define this if you want to add support for a worker thread */
#define USE_WORKER 1

/* USE_BIGNUM - Enable BigFloat, BigDecimal by default */
/* #undef USE_BIGNUM */

/* DUMP_ATOMS - Define this if you want to dump atoms when freeing context */
/* #undef DUMP_ATOMS */

/* DUMP_SHAPES - Define this if you want to dump shapes when freeing context */
/* #undef DUMP_SHAPES */

/* DUMP_OBJECTS - Define this if you want to dump objects when freeing context */
/* #undef DUMP_OBJECTS */

/* DUMP_MEM - Define this if you want to dump memory when freeing context */
/* #undef DUMP_MEM */

/* DUMP_BYTECODE - Define this if you want to be able to dump bytecode */
/* #undef DUMP_BYTECODE */

/* DUMP_FREE - Define this if you want to dump a message on freeing objects */
/* #undef DUMP_FREE */

/* DUMP_GC - Define this if you want to dump a message on garbage collection */
/* #undef DUMP_GC */

/* DUMP_GC_FREE - Define this if you want to dump a message when the garbage collector free's a resource */
/* #undef DUMP_GC_FREE */

/* DUMP_LEAKS - Define this if you want to dump memory leaks */
/* #undef DUMP_LEAKS */

/* DUMP_MODULE_RESOLVE - Define this if you want to debug module resolution */
/* #undef DUMP_MODULE_RESOLVE */

/* DUMP_PROMISE - Define this if you want to debug promises */
/* #undef DUMP_PROMISE */

/* DUMP_READ_OBJECT - Define this if you want to debug binary object reader */
/* #undef DUMP_READ_OBJECT */

/* HAVE_GETHOSTBYNAME - Define this if you have the gethostbyname() function */
#define HAVE_GETHOSTBYNAME 1

/* HAVE_INET_PTON - Define this if you have the inet_pton() function */
#define HAVE_INET_PTON 1

/* HAVE_INET_ATON - Define this if you have the inet_aton() function */
/* #undef HAVE_INET_ATON */

/* HAVE_NETDB_H - Define this if you have the netdb.h header file */
#define HAVE_NETDB_H 1

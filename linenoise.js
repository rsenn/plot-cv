import { call, define, dlopen, dlsym, RTLD_NOW, toString } from 'ffi';

const liblinenoise = dlopen('/usr/local/lib/x86_64-linux-gnu/liblinenoise.so', RTLD_NOW);
console.log('liblinenoise', liblinenoise);

/* void linenoiseSetCompletionCallback(linenoiseCompletionCallback* fn)  */
define('linenoiseSetCompletionCallback', dlsym(liblinenoise, 'linenoiseSetCompletionCallback'), null, 'void', 'void *');

export function linenoiseSetCompletionCallback(fn) {
  return call('linenoiseSetCompletionCallback', fn);
}

/* void linenoiseAddCompletion(linenoiseCompletions* lc, const char* str)  */
define('linenoiseAddCompletion', dlsym(liblinenoise, 'linenoiseAddCompletion'), null, 'void', 'void *', 'string');

export function linenoiseAddCompletion(lc, str) {
  return call('linenoiseAddCompletion', lc, str);
}

/* char* linenoise(const char* prompt)  */
define('linenoise', dlsym(liblinenoise, 'linenoise'), null, 'string', 'string');

export function linenoise(prompt) {
  let r = call('linenoise', prompt);
  if(r > 0) r = toString(r);
  return r;
}

/* int linenoiseHistoryAdd(const char* line)  */
define('linenoiseHistoryAdd', dlsym(liblinenoise, 'linenoiseHistoryAdd'), null, 'int', 'string');

export function linenoiseHistoryAdd(line) {
  return call('linenoiseHistoryAdd', line);
}

/* int linenoiseHistorySetMaxLen(int len)  */
define('linenoiseHistorySetMaxLen', dlsym(liblinenoise, 'linenoiseHistorySetMaxLen'), null, 'int', 'int');

export function linenoiseHistorySetMaxLen(len) {
  return call('linenoiseHistorySetMaxLen', len);
}

/* int linenoiseHistorySave(const char* filename)  */
define('linenoiseHistorySave', dlsym(liblinenoise, 'linenoiseHistorySave'), null, 'int', 'string');
export function linenoiseHistorySave(filename) {
  return call('linenoiseHistorySave', filename);
}

/* int linenoiseHistoryLoad(const char* filename)  */
define('linenoiseHistoryLoad', dlsym(liblinenoise, 'linenoiseHistoryLoad'), null, 'int', 'string');
export function linenoiseHistoryLoad(filename) {
  return call('linenoiseHistoryLoad', filename);
}

/* void linenoiseSetMultiLine(int ml)  */
define('linenoiseSetMultiLine', dlsym(liblinenoise, 'linenoiseSetMultiLine'), null, 'void', 'int');
export function linenoiseSetMultiLine(ml) {
  return call('linenoiseSetMultiLine', ml);
}

/* void linenoisePrintKeyCodes()  */
define('linenoisePrintKeyCodes', dlsym(liblinenoise, 'linenoisePrintKeyCodes'), null, 'void');
export function linenoisePrintKeyCodes() {
  return call('linenoisePrintKeyCodes');
}

export default {
  SetCompletionCallback: linenoiseSetCompletionCallback,
  AddCompletion: linenoiseAddCompletion,
  linenoise,
  HistoryAdd: linenoiseHistoryAdd,
  HistorySetMaxLen: linenoiseHistorySetMaxLen,
  HistorySave: linenoiseHistorySave,
  HistoryLoad: linenoiseHistoryLoad,
  SetMultiLine: linenoiseSetMultiLine,
  PrintKeyCodes: linenoisePrintKeyCodes
};
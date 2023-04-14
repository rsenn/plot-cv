import tokenize from './tokenize.js';
import { abbreviate } from './lib/misc.js';
import { ReadFile, WriteFile } from './io-helpers.js';

const consoleOpts = {
  depth: Infinity,
  compact: 2,
  hideKeys: [],
  maxArrayLength: Infinity
};

function main(...args) {
  console.options = consoleOpts;

  let code = ReadFile(args[0] ?? 'pa_devs.c', 'utf-8');
  console.log(abbreviate(code));

  let i = 0;

  for(let token of tokenize(code)) {
    console.log(`token #${i}`, token);
    i++;
  }
}

main(...scriptArgs.slice(1));

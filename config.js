import { ReadFile, WriteFile } from './io-helpers.js';
import fs from 'fs';
import process from 'process';
import { memoize } from './lib/misc.js';

let basename = memoize(() => process.argv[1].replace(/\.js$/, ''));

export function SaveConfig(configObj) {
  configObj = Object.fromEntries(Object.entries(configObj).map(([k, v]) => [k, +v]));

  let ret = WriteFile(process.argv[1].replace(/\.js$/, '.config.json'), JSON.stringify(configObj, null, 2) + '\n');
  console.log(`Saved config to '${basename() + '.config.json'}'`, inspect(configObj, { compact: false }));
  return ret;
}

export function LoadConfig(name = process.argv[1].replace(/\.js$/, '.config.json')) {
  let str = ReadFile(name, 'utf-8');
  let configObj = JSON.parse(str ?? '{}');

  configObj = Object.fromEntries(
    Object.entries(configObj)
      .map(([k, v]) => [k, +v])
      .filter(([k, v]) => !isNaN(v))
  );
  console.log('LoadConfig:', configObj);
  return configObj;
}

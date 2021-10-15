import * as std from 'std';
import inspect from 'inspect';
import { memoize } from './lib/misc.js';
import { SaveConfig,  LoadConfig } from './config.js';

let basename = memoize(() => process.argv[1].replace(/\.js$/, ''));

export function SaveConfig(configObj) {
  configObj = Object.fromEntries(Object.entries(configObj).map(([k, v]) => [k, +v]));
  let file = std.open(basename() + '.config.json', 'w+b');
  file.puts(JSON.stringify(configObj, null, 2) + '\n');
  file.close();
  console.log(
    `Saved config to '${basename() + '.config.json'}'`,
    inspect(configObj, { compact: false })
  );
}

export function LoadConfig() {
  let str = std.loadFile(basename() + '.config.json');
  let configObj = JSON.parse(str || '{}');

  configObj = Object.fromEntries(
    Object.entries(configObj)
      .map(([k, v]) => [k, +v])
      .filter(([k, v]) => !isNaN(v))
  );
  console.log('LoadConfig:', inspect(configObj, { compact: false }));
  return configObj;
}

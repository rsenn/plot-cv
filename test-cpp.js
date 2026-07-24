import { readFileSync } from 'fs';
import * as fs from 'fs';
import { randInt, abbreviate } from 'util';
import cpp from './lib/cpp.js';
import * as path from 'path';

const includeDirs = [process.env.TERMUX_PREFIX + '/lib/gcc/x86_64-linux-gnu/16/include', process.env.TERMUX_PREFIX + '/include/x86_64-linux-gnu', '/opt/diet/include', '.'];

const FindIncludeFunc = source => {
  const dirs = [path.dirname(source), ...includeDirs];
  return name => {
    for(let dir of dirs) {
      let file = path.join(dir, name);
      if(path.exists(file)) return file;
    }
  };
};

const getSource = () => sources[randInt(0, sources.length - 1, prng)];

function StripPP(code) {
  return code
    .split(/\n/g)
    .filter(line => !/^\s*#/.test(line))
    .join('\n');
}

function main(...args) {
  const file = args[0] || getSource();
  console.log('Source file:', file);

  let cmd = ['/usr/lib/gcc/x86_64-linux-gnu/10/cc1', '-E', ...includeDirs.map(dir => `-I${dir}`), file];
  console.log('cmd:', cmd.join(' '));

  console.log('Source file:', file);
  const src = readFileSync(file, 'utf-8');
  const findInclude = FindIncludeFunc(file);
  let code;
  const pp = (globalThis.pp = cpp({
    includeFunc(file, system, resolve) {
      console.log('includeFunc', console.config({ compact: true }), {
        file,
        system,
      });

      file = findInclude(file);

      const code = fs.readFileSync(file, 'utf-8');
      resolve(code);
    },
    completionFunc(text, arr, state) {
      code = text;
    },
    errorFunc(error, stack) {
      //console.log('errorFunc', { error, stack });
      throw Object.assign(new Error(error), stack ? { stack } : {});
      std.exit(1);
    },
    warnFunc(warning) {
      //console.log('warnFunc', { warning });
    },
  }));
  pp.define('__WORDSIZE', 64);
  pp.define('__STDC_LIMIT_MACROS', 1);
  pp.define('__STDC_CONSTANT_MACROS', 1);
  pp.define('__SIZE_TYPE__', 'unsigned long');
  pp.define('__PTRDIFF_TYPE__', 'long');
  pp.define('__GNUC_PREREQ(x,y)', 1);

  const e = pp.run(src);
  console.log(e);

  os.kill(os.getpid(), os.SIGUSR1);
}

main(...scriptArgs.slice(1));

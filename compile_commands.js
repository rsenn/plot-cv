import * as fs from 'fs';
import * as path from 'path';
import { RecursiveDirIterator } from './dir-helpers.js';
import { PathTransformer, ArgumentType, ArgumentIs, CommandType, CompileCommand, MakeCommand, MakeCommands } from './lib/compileCommand.js';
import { arraysInCommon, define, mapFunction, types } from './lib/misc.js';
import { Console } from 'console';

const commands = (globalThis.commands = []);
const cmdMap = (globalThis.cmdMap = mapFunction(new WeakMap()));

const getMany =
  fn =>
  (...args) =>
    args.length == 1 && Array.isArray(args[0])
      ? (function* iter(arr) {
          for(let item of arr) yield fn(item) ?? item;
        })(args[0])
      : fn(...args);

const runGenerator =
  fn =>
  (...args) => {
    let result = fn.call(null, ...args);
    if(types.isIterator(result)) result = [...result];
    return result;
  };

const targetMap = (globalThis.targetMap = runGenerator(getMany(mapFunction(new Map()))));
const sourcesMap = (globalThis.sourcesMap = outputFile => {
  let { deps } = targetMap(outputFile);
  deps = targetMap(deps);
  deps = deps.flatMap(dep => dep.sources).filter(s => typeof s == 'string');
  return deps;
});

const depMap = (globalThis.depMap = inputFile =>
  Object.values(commands.reduce((acc, cmd) => ([...(cmd.sources ?? cmd.dependencies)].indexOf(inputFile) != -1 ? ((acc[cmd.outputFile] ??= cmd), acc) : acc), {})));

const transformMap = (globalThis.transformMap = (input, output, map) => {
  let fn = mapFunction(map);
  return (...args) => {
    args = args.map(input);
    let result = fn.call(null, ...args);
    if(args.length < 2) result = output(result);
    return result;
  };
});

const workDir = (globalThis.workDir = workDir => {
  if(path.isRelative(workDir)) workDir = path.absolute(workDir);
  return {
    toAbsolute(p) {
      if(path.isRelative(p)) p = path.join(this?.workDir ?? workDir, p);
      return p;
    },
    toRelative(p) {
      if(Array.isArray(p)) return p.map(toRel);
      function toRel(p) {
        return path.isAbsolute(p) ? path.relative(workDir, p) : p;
      }
      return toRel(p);
    },
  };
});

const relativeMap = (globalThis.relativeMap = (relativeTo, map) => {
  let { toAbsolute, toRelative } = workDir(path.absolute(relativeTo));

  return transformMap(toAbsolute, toRelative, map);
});

const binutils = (globalThis.binutils = {
  nm: (...args) =>
    [...Shell(`nm ${args.join(' ')}`).matchAll(/[^\n]+/g)]
      .map(m => m[0].split(/:/))
      .filter(entry => entry.length > 1)
      .map(([file, data]) => [file, data.slice(0, 16), data.slice(17, 18), data.slice(19)])
      .reduce((acc, [file, addr, type, name]) => ((acc[file] ??= []), acc[file].push({ addr: parseInt(addr.trim() || '0'), type, name }), acc), {}),
});

Object.assign(globalThis, { PathTransformer });

function main(...arglist) {
  const { stdout, stderr } = process;
  globalThis.console = new Console({
    stdout,
    stderr,
    inspectOptions: {
      colors: true,
      depth: Infinity,
      compact: false,
      maxArrayLength: Infinity,
      maxStringLength: Infinity,
    },
  });
  let file = arglist[0] ?? '/home/roman/Projects/plot-cv/quickjs/qjs-modules/build/x86_64-linux-debug/compile_commands.json';

  console.log('file', file);

  let builddir = path.dirname(file);
  let json = fs.readFileSync(file, 'utf-8');
  let compileCommands = JSON.parse(json);
  let prevDirectory;
  let directories = new Set(),
    workingDir = process.cwd();

  for(let { directory, command } of compileCommands) {
    let cmd = MakeCommand(command, directory);

    /*if(-1 == commands.findIndex(c => c.outputFile == cmd.outputFile))*/ {
      commands.push(cmd);

      targetMap(cmd.outputFile, cmd);
      cmdMap(cmd, {
        string: command,
        file,
        get dependencies() {
          return targetMap(cmd.dependencies);
        },
      });
    }

    directories.add(directory);
    prevDirectory = directory;
  }
  let common = arraysInCommon(commands).filter((arg, i) => i > 0 && ['mode', 'output'].indexOf(ArgumentType(arg)) == -1);

  let i = 0;
  for(let cmd of commands) {
    //cmd.remove(...common);
    let { program, output, source } = cmd;

    targetMap(cmd.outputFile, cmd);

    ++i;
  }
  let { program } = commands[0];

  common.unshift(program);
  let newCmd = common.join(' ');

  let commonCmd = (globalThis.commonCmd = new CompileCommand(newCmd));

  const { defines, includePaths, flags } = commonCmd;

  define(globalThis, {
    CompileCommand,
    ArgumentType,
    ArgumentIs,
    CommandType,
    get CFLAGS() {
      return [...defines.map(d => '-D' + d), ...includePaths.map(i => '-I' + i), ...flags];
    },
  });

  let linkFiles = [...directories].flatMap(dir => [...RecursiveDirIterator(dir, (entry, file) => /link\.txt$/.test(file))]);

  console.log('linkFiles', linkFiles);

  for(let file of linkFiles) {
    const string = fs.readFileSync(file, 'utf-8');
    let [cmd] = MakeCommands(string, file.replace(/\/CMakeFiles\/.*/g, ''));

    const { output, objects } = cmd;

    /*if(-1 == commands.findIndex(c => c.outputFile == cmd.outputFile))*/ {
      commands.push(cmd);
      targetMap(cmd.outputFile, cmd);
      cmdMap(cmd, {
        string,
        file,
        get dependencies() {
          return targetMap(cmd.dependencies);
        },
      });
    }
    console.log('cmd', cmd);

    (globalThis.linkCommands ??= []).push(cmd);
  }

  console.log('commands', commands);

  process.kill(process.pid, 10);
  //startInteractive();
}

try {
  main(...process.argv.slice(2));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  process.exit(1);
} finally {
  console.log('SUCCESS');
}

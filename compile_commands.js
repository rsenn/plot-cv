import { Console } from 'console';
import * as fs from 'fs';
import * as path from 'path';
import * as util from './lib/misc.js';
import inspect from 'inspect';
import child_process from './lib/childProcess.js';
import { ReadFile, ReadJSON, ReadBJSON, FdReader } from './io-helpers.js';
import { CompileCommand, MakeCommand, ArgumentType } from './lib/compileCommand.js';

function main(...arglist) {
  let { width, height } = util.getScreenSize({});
  globalThis.console = new Console(process.stdout, {
    inspectOptions: {
      colors: true,
      depth: Infinity,
      compact: false,
      breakLength: width,
      maxArrayLength: Infinity,
      maxStringLength: Infinity
    }
  });
  let json = ReadFile(arglist[0] ?? '/home/roman/Projects/plot-cv/quickjs/qjs-modules/build/x86_64-linux-debug/compile_commands.json', 'utf-8');
  let compileCommands = JSON.parse(json);
  let prevDirectory;
  let commands = [],
    directories = [],
    workingDir = path.getcwd();

  for(let { directory, command } of compileCommands) {
    let cmd = MakeCommand(command);
    commands.push(cmd);
    directories.push(path.relative(workingDir, directory));
    prevDirectory = directory;
  }

  let common = util.arraysInCommon(commands).filter((arg, i) => i > 0 && ['mode', 'output'].indexOf(ArgumentType(arg)) == -1);

  let i = 0;
  for(let cmd of commands) {
    cmd.remove(...common);

    // let sources = cmd.sources.map(s => '\x1b[32m' + path.relative(workingDir, s) + '\x1b[0m');

    {
      let { program, output, source } = cmd;
      output = path.join(directories[i], output);
      console.log('cmd', { dir: directories[i], output });
      cmd.output = path.relative(directories[i], output);

      cmd.program = path.basename(program);

      if(source) cmd.source = path.relative(workingDir, source);
    }

    let idx = cmd.findIndex(a => /^-(o|c|S|E)/.test(a));
    // cmd.splice(idx, 0, '$(CFLAGS)');

    {
      let { program, output, source, flags } = cmd;
      console.log('cmd', inspect(cmd, { ...console.options, compact: 2 }));
    }
    //    std.puts(cmd.toString('' && '\\\n\t') + '\n');
    ++i;
  }
  let { program } = commands[0];

  common.unshift(program);
  let newCmd = common.join(' ');

  let commonCmd = new CompileCommand(newCmd);

  const { defines, includes, flags } = commonCmd;

  const CFLAGS = [...defines.map(d => '-D' + d), ...includes.map(i => '-I' + i), ...flags];
  console.log('CFLAGS', CFLAGS);

  const linkFiles = commands.map(({ output, directory }, i) => path.join(directories[i], output.replace(/(\.dir)\/.*/g, '$1/link.txt')));

  for(let linkFile of linkFiles) {
    //console.log('linkFile', linkFile);

    let cmd = MakeCommand(ReadFile(linkFile));

    //console.log('cmd', console.config({ compact: 2 }), cmd);
    (globalThis.linkCommands ??= []).push(cmd);
  }

  util.startInteractive();
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}

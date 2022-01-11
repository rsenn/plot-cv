import { Console } from 'console';
import * as fs from 'fs';
import * as path from 'path';
import * as util from 'util';
import inspect from 'inspect';
import child_process from './lib/childProcess.js';
import { ReadFile, ReadJSON, ReadBJSON, ReadDirRecursive, FdReader } from './io-helpers.js';
import { CompileCommand, ArgumentType } from './lib/compileCommand.js';

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
  let json = fs.readFileSync(
    '/home/roman/Projects/plot-cv/quickjs/qjs-modules/build/x86_64-linux-profile/compile_commands.json',
    'utf-8'
  );
  let compileCommands = JSON.parse(json);
  let prevDirectory;
  let commands = [],
    directories = [],
    workingDir = path.getcwd();

  //console.log('workingDir', workingDir);
  for(let { directory, command } of compileCommands) {
    let cmd = new CompileCommand(command);
    commands.push(cmd);
    directories.push(directory);
    prevDirectory = directory;
  }

  let common = util
    .arraysInCommon(commands)
    .filter((arg, i) => i > 0 && ['mode', 'output'].indexOf(ArgumentType(arg)) == -1);
  // console.log('common', common);
  let i = 0;
  for(let cmd of commands) {
    cmd.remove(...common);
    let sources = cmd.sources.map(s => '\x1b[32m' + path.relative(s, workingDir) + '\x1b[0m');
    //console.log('compileCommand.sources', ...sources);

    {
      let { program, output, sources } = cmd;
      output = path.join(directories[i], output);
      output = path.relative(output, workingDir);

      cmd.program = path.basename(program);
      cmd.output = path.basename(output);

      cmd.sources = sources.map(src => path.relative(src, workingDir));
    }

    let idx = cmd.findIndex(a => /^-(o|c|S|E)/.test(a));
    cmd.splice(idx, 0, '$(CFLAGS)');

    std.puts(cmd.toString('' && '\\\n\t') + '\n');
    ++i;
  }
  let { program } = commands[0];
  /*  console.log('program', program);
  console.log('common', common.join(' \\\n\t'));*/

  common.unshift(program);
  let newCmd = common.join(' ');
  //console.log('newCmd', newCmd);

  let commonCmd = new CompileCommand(newCmd);
  //  console.log('commonCmd', commonCmd.toString(' \\\n\t'));

  const { defines, includes, flags } = commonCmd;

  //console.log('commonCmd', { defines, includes, flags });
  const CFLAGS = [...defines.map(d => '-D' + d), ...includes.map(i => '-I' + i), ...flags];
  console.log('CFLAGS', CFLAGS);
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}

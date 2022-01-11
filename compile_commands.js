import { Console } from 'console';
import * as fs from 'fs';

function main(...args) {
  globalThis.console = new Console(process.stderr, {
    inspectOptions: {
      colors: true,
      depth: 2,
      breakLength: 160,
      maxStringLength: Infinity,
      maxArrayLength: Infinity,
      compact: 1
    }
  });
  let json = fs.readFileSync(
    '/home/roman/Projects/plot-cv/quickjs/qjs-modules/build/x86_64-linux-profile/compile_commands.json',
    'utf-8'
  );
  let compileCommands = JSON.parse(json);
  let prevDirectory;

  for(let { directory, command } of compileCommands) {
    if(prevDirectory != directory) std.puts(`cd ${directory}\n`);

    //console.log('compileCommand',{directory,command});
    std.puts(command + '\n');
    prevDirectory = directory;
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}

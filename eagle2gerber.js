import { Console } from 'console';
import { EagleToGerber, ExecTool } from './pcb-conversion.js';

let optionsArray = [{ front: true }, { back: true }, { side: 'outline' }, { drill: true }];

function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: Infinity
  });

  if(args.length == 0) args = ['/dev/stdin'];

  for(let arg of args) {
    let files = [];
    for(let options of optionsArray) {
      let cmd = EagleToGerber(arg, options);
      let outputFile = cmd[cmd.indexOf('-o') + 1];
      files.push(outputFile);

      //  console.log(`Generating '${outputFile}'`);
      console.log(`Command: ${cmd.join(' ')}`);

      let result = ExecTool(...cmd);
      console.log(`Result:`, result);
    }
    console.log(`Generated files:\n${files.join('\n')}`);
  }
}

main(...scriptArgs.slice(1));

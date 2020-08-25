import Util from './lib/util.js';

 async function SetupConsole(options) {
  let ret;
  try {
    Util.tryCatch(() => Error.stackTraceLimit = 1000);

    const { Console } = await import('console');
    ret = new Console({
      stdout: process.stdout,
      stderr: process.stderr,
      inspectOptions: { depth: 2, colors: true, ...options }
    });
  } catch(err) {}
  return Util.tryCatch(() => global.console = ret);
}

export const ConsoleSetup = Util.once(opts => SetupConsole(opts));

export default ConsoleSetup;
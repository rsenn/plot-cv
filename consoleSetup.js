import Util from './lib/util.js';

export async function ConsoleSetup(options) {
  let ret;
  try {
    Util.tryCatch(() => (Error.stackTraceLimit = 1000));

    const { Console } = await import('console');
    ret = new Console({
      stdout: process.stdout,
      stderr: process.stderr,
      inspectOptions: { depth: 2, colors: true, ...options }
    });
  } catch(err) {}
  return Util.tryCatch(() => (globalThis.console = ret));
}

export const ConsoleOnce = Util.once(opts => ConsoleSetup(opts));

export default ConsoleOnce;

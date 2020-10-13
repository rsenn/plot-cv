import Util from './lib/util.js';

export async function ConsoleSetup(opts = {}) {
  let ret;
  Util.tryCatch(() => (Error.stackTraceLimit = 1000));
  const proc = await Util.tryCatch(async () => await import('process'));
  const defaultBreakLength =
    proc && proc.stdout && proc.stdout.isTTY ? proc.stdout.columns || proc.env.COLUMNS : Infinity;
  const {
    depth = 2,
    colors = await Util.isatty(1),
    breakLength = defaultBreakLength,
    maxArrayLength = Infinity,
    ...options
  } = opts;
  try {
    //  const { Console } = await import('console');
    const Console = await import('console').then(module => module.Console);
    console.log('Console:', Console);
    ret = new Console({
      stdout: proc.stdout,
      stderr: proc.stderr,
      inspectOptions: { depth, colors, breakLength, maxArrayLength, ...options }
    });
    ret.colors = colors;
    ret.depth = depth;
  } catch(err) {}

  if(ret) return Util.tryCatch(() => (globalThis.console = ret));
}

export const ConsoleOnce = Util.once(opts => ConsoleSetup(opts));

export default ConsoleOnce;

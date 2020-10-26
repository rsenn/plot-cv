import Util from './lib/util.js';

export async function ConsoleSetup(opts = {}) {
  let ret;
  Util.tryCatch(() => (Error.stackTraceLimit = 1000));
  const proc = await Util.tryCatch(async () => await import('process'),
    ({ stdout, env }) => ({ stdout, env }),
    () => ({ stdout: {}, env: {} })
  );
  const defaultBreakLength =
    (proc && proc.stdout && proc.stdout.isTTY && proc.stdout.columns) || proc.env.COLUMNS || 80; // Infinity;
  const {
    depth = 2,
    colors = await Util.isatty(1),
    breakLength = defaultBreakLength,
    maxArrayLength = Infinity,
    ...options
  } = opts;
  ret = await Util.tryCatch(async () => {
      const Console = await import('console').then(module => module.Console);
      ret = new Console({
        stdout: proc.stdout,
        stderr: proc.stderr,
        inspectOptions: { depth, colors, breakLength, maxArrayLength, ...options }
      });
      ret.colors = colors;
      ret.depth = depth;
      return ret;
    },
    c => c,
    () => console
  );

  for(let method of ['error', 'warn', 'debug']) {
    if(!(method in ret)) ret[method] = ret.log;
  }

  if(ret) return Util.tryCatch(() => (globalThis.console = ret));
}

export const ConsoleOnce = Util.once(opts => ConsoleSetup(opts));

export default ConsoleOnce;

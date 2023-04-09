
class MyObj {
  a = 1;
  b = null;
  c = NaN;

  [Symbol.for('nodejs.util.inspect.custom')](...args) {
    console.log('inspect args=', args);
    return '';
  }
}

async function main(...args) {

  console.log('inspect:', globalThis.ObjectInspect);
  console.log('MyObj:', { test: [1, 2, 3, new MyObj()] });

  console.log('stackFrame:', Util.getStackFrame(1));
  let st = Util.getCallerStack();
  console.log('getCallerStack:', st);
  /*console.log('test:', 1);*/
}

main(...scriptArgs.slice(1));

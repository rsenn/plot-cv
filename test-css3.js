import { selector } from './grammar-css3.js';
import { Console } from 'console';
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
  let input =
    '#jsc_c_bsh > div:nth-child(21) > div > div > div.j83agx80 > div.j83agx80.buofh1pr.rl25f0pe.o6b9zlra > div.ns4p8fja.j83agx80.cbu4d94t.a6sixzi8.bkfpd7mw.a1xu1aao > div.j83agx80.k4urcfbm > div.j83agx80.cbu4d94t.d2edcug0.l9j0dhe7.aovydwv3 > span > div > div.cxgpxx05.d1544ag0.sj5x9vvc.tw6a2znq.l9j0dhe7.ni8dbmo4.stjgntxs.e72ty7fz.qlfml3jp.inkptoze.qmr60zad.jm1wdb64.qv66sw1b.ljqsnud1.odn2s2vf.tkr6xdv7 > div > div';

  let result = selector(input, 0);
  /*  console.log('str,pos', { str, pos });*/
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
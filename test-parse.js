import { ReadFile, WriteFile } from './io-helpers.js';
import { Grammar } from './lib/parse/grammar.js';
import * as path from './lib/path.js';
import { Console } from 'console';
function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  WriteFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
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

  let [
    filename = './lib/grammars/bnf.g4',
    input = '#jsc_c_bsh > div:nth-child(21) > div > div > div.j83agx80 > div.j83agx80.buofh1pr.rl25f0pe.o6b9zlra > div.ns4p8fja.j83agx80.cbu4d94t.a6sixzi8.bkfpd7mw.a1xu1aao > div.j83agx80.k4urcfbm > div.j83agx80.cbu4d94t.d2edcug0.l9j0dhe7.aovydwv3 > span > div > div.cxgpxx05.d1544ag0.sj5x9vvc.tw6a2znq.l9j0dhe7.ni8dbmo4.stjgntxs.e72ty7fz.qlfml3jp.inkptoze.qmr60zad.jm1wdb64.qv66sw1b.ljqsnud1.odn2s2vf.tkr6xdv7 > div > div'
  ] = args;
  let basename = path.basename(filename, path.extname(filename));

  let src = ReadFile(filename, 'utf-8');
  console.log('src:', src);

  let grammar = new Grammar(src, filename);
  grammar.debug = true;
  grammar.parse();
  console.log('grammar:', grammar);
  const outfile = `grammar-${basename}.js`;
  WriteFile(outfile, grammar.generate('./lib/parse/'));

  import(outfile).then(grammar => {
    const { selector } = grammar;
    console.log('selector', selector);

    let [str, pos] = selector(input, 0);
    console.log('str,pos', { str, pos });
  });
  /*
  let a = [];
  for(let [name, rule] of grammar.rules) {
    a.push(rule.toCowbird(a, name));
  }

  let cowbirdGrammar = grammar.toCowbird();
  console.log('cowbird:', cowbirdGrammar);

  let data = ReadFile('../pictest/build/mplab/7segtest-16f876a-xc8-debug.mcp');
  console.log('data:', abbreviate(data, 100));
  let parser = new Cowbird(cowbirdGrammar, 'ini', true);
  console.log('parser:', parser);

  let result = parser.parse(data);
  console.log('result:', result);
*/
  return;
}

main(...scriptArgs.slice(1));
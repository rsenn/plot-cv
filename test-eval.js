const code = `
 let testObj = {};

let testValues = [1, 2, 3, 4, 5];

trkl.bind(testObj, 'prop1', value => (value === undefined ? testValues[0] : (testValues[0] = value)));

console.log('testObj.prop1', testObj.prop1);
testObj.prop1 = 'a';
console.log('testObj.prop1', testObj.prop1);
console.log('testValues', testValues);
 `;

import { Environment, ECMAScriptParser, Printer } from './lib/ecmascript.js';
import Util from './lib/util.js';
import trkl from './lib/trkl.js';
import PortableFileSystem from './lib/filesystem.js';

let filesystem;

async function main(args) {
  filesystem = await PortableFileSystem();

  let file = 'test-trkl.js';

  let data = /* code ||*/ filesystem.readFile(file).toString();

  let parser = new ECMAScriptParser(data, code ? process.argv[1].replace(/.*\//g, '') : file, true);
  let ast = parser.parseProgram();
  let printer = new Printer({ indent: 2 });

  parser.addCommentsToNodes(ast);

  let env = new Environment([
    {
      Symbol: { species: Symbol.for('species') },
      console: {
        log(...args) {
          console.debug('console.log(',
            ...args.map(arg => `'${arg}'`).reduce((acc, arg) => (acc ? [...acc, ',', arg] : [arg]), null),
            ')'
          );
        }
      },
      trkl
    }
  ]);

  await Util.safeCall(() => {
    let iter = env.generate(ast);
    console.log('iter:', iter);

    for(let it of iter()) console.info('it:', it);
  });

  let output = printer.print(ast);

  let outputFile = 'output.es';
  console.log(`wrote '${outputFile}'`, await filesystem.writeFile('output.es', output));
}
try {
  Util.callMain(main, true);
} catch(error) {
  console.error('ERROR:', error);
}

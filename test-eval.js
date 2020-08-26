import { Environment, ECMAScriptParser, Printer } from './lib/ecmascript.js';
import Util from './lib/util.js';

function main(args) {
  let file = 'lib/geom/point.js';

  let data = filesystem.readFile(file).toString();

  let env = new Environment([
    {
      Symbol: { species: Symbol.for('species') }
    }
  ]);
  let parser = new ECMAScriptParser(data, file);
  let ast = parser.parseProgram();
  let printer = new Printer({ indent: 2 });

  parser.addCommentsToNodes(ast);

  let output = printer.print(ast);

  console.log('output:', output);

  let iter = env.generate(ast);
}

try {
  main(Util.getArgs());
} catch(error) {
  Util.putError(error);
}

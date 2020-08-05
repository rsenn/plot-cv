import { Environment, ECMAScriptParser, Printer } from './lib/ecmascript.js';
import Util from './lib/util.js';
import fs from 'fs';


function main(args) {
  let file = './lib/dom/element.js';

      let data = fs.readFileSync(file).toString();

  let env = new Environment();
  let parser =  new ECMAScriptParser(data, file);
      let ast = parser.parseProgram();
let printer = new Printer({ indent: 2 });
      parser.addCommentsToNodes(ast);
      let output = printer.print(ast);

      console.log("output:",output);
}

try {
main(process.argv.slice(2));
}catch(error) {
  Util.putError(error);
}

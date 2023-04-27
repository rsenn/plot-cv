import { Console } from 'console';
import { Circuit, CircuitFileParser, CircuitFileWriter } from './circuit.js';

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxArrayLength: Infinity,
      colors: true,
      compact: 0,
      customInspect: true
    }
  });

  let parser = new CircuitFileParser();
  let circuit = parser.parse('/home/roman/Dokumente/Electronics/example.circuit');
  let { comments, elements } = circuit;
  let { lines } = parser;
  //console.log('lines', lines);
  //console.log('elements', elements);
  /*console.log(
    'comments',
    console.config({ compact: 2 }),
    elements.reduce((acc, el) => acc.concat([comments.get(el), el]), []).filter(n => n !== undefined)
  );*/
  let writer = new CircuitFileWriter(circuit);
  let f = openSync('out.circuit', 'w+');

  writer.write(s => f.puts(s));
  closeSync(f);
}

main(...scriptArgs.slice(1));

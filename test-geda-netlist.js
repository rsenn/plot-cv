import gedaNetlistGrammar from './grammar-geda-netlist.js';
import { Point, Size } from './lib/geom.js';
import * as path from './lib/path.js';
import inspect from 'inspect';
function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
  let xy = new Point();
  let size = new Size(128, 128);
  let maxWidth = 1360;
  let newSize;
  let spacing = 32;
  let count = 0;
  let iconSize, iconAspect;

  for(let filename of args) {
    let src = filesystem.readFileSync(filename);
    let base = path.basename(filename, /\.[^.]*$/);

    //console.log('src:', escape(src));
    const result = gedaNetlistGrammar.geda_netlist(src, 0);
    let [done, data, pos] = result;

    let [components, nets] = data;

    const findFirst = arr => {
      for(let i = 0; i < arr.length; i++) if(arr[i] != '') return i;
      return arr.length;
    };
    const findLast = (arr, start = 0) => {
      for(let i = start; i < arr.length; i++) if(arr[i] === '') return i;
      return arr.length;
    };

    const cleanArray = arr =>
      arr
        .map(c => [[''], ...c[0]])
        .map(c => c.flat(2))
        .map(c => c.slice(findFirst(c)))
        .map(c =>
          c[1] !== ''
            ? c
            : c.reduce((acc, item, idx) => {
                if(idx % 2 == 0) acc.push(item);
                else acc[acc.length - 1] += item;
                return acc;
              }, [])
        )
        .map(c => c.slice(0, findLast(c)));
    components = Object.fromEntries(cleanArray(components).map(([name, ...rest]) => [name, rest]));
    nets = Object.fromEntries(cleanArray(nets).map(([name, ...rest]) => [name, rest]));
    console.log('nets:', nets);
    console.log('components:', components);

    let output = { components, nets };

    let json = inspect(output, {
      multiline: true,
      depth: 2,
      json: true,
      quote: '"'
    }); //JSON.stringify(output, null, 2);

    WriteFile(base + '.json', json);
  }
}

main(...scriptArgs.slice(1));
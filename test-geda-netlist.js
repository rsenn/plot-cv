import gedaNetlistGrammar from './grammar-geda-netlist.js';
import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import { Point, Size, Rect, BBox } from './lib/geom.js';
import deep from './lib/deep.js';
import ConsoleSetup from './lib/consoleSetup.js';
import tXml from './lib/tXml.js';
import { XPath } from './lib/xml.js';
import { toXML } from './lib/json.js';

let filesystem;

function WriteFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));
  await ConsoleSetup({ depth: Infinity });

  let xy = new Point();
  let size = new Size(128, 128);
  let maxWidth = 1360;
  let newSize;
  let spacing = 32;
  let count = 0;
  let iconSize, iconAspect;

  for(let filename of args) {
    let src = filesystem.readFile(filename);
    let base = path.basename(filename, /\.[^.]*$/);

    console.log('src:', Util.escape(src));
    const result = gedaNetlistGrammar.geda_netlist(src, 0);
    let [done, data, pos] = result;

    let  [ components,nets] = data;

      nets = Object.fromEntries(nets.map(net => net.flat(2)).map(([name, ...connections]) => [name,connections]));
   console.log('nets:', nets);
       // components = (components.map(component => component.flat(2)).map(([ name, footprint, value, ...rest]) => ({name,footprint,value})));
        components = Object.fromEntries(components.map(component => component.flat(2)).map(([ name, footprint, value, ...rest]) => [name, [footprint,value]]));
  console.log('components:', components);


  let output = { components, nets };

  let json = Util.toString(output, { multiline: true, depth:2, json: true, quote: '"'}); //JSON.stringify(output, null, 2);

  WriteFile(base+'.json', json);
}
}

Util.callMain(main, true);

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

 console.log('src:', Util.escape(src));
 const result  = gedaNetlistGrammar.geda_netlist(src, 0);
    let [done, data, pos] = result;
 console.log('result:', result);
}
/*
    let createMap = entries =>   new Map(entries);

    let sections = data[0].reduce((acc, sdata) => {
      console.log('sdata:', sdata);
      return { ...acc, [sdata[0]]: createMap(sdata[1] || []) };
    }, {});

    const flat = deep.flatten(sections,
      new Map(),
      k => k.length > 0,
      (k, v) => [k.slice(1), v]
    );
    console.log('flat:', flat);
  }
   */

}

Util.callMain(main, true);

import { EagleDocument, Renderer } from './lib/eagle.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { render, Component } from './lib/preact.mjs';
import { ColoredText } from './lib/color/coloredText.js';
import { BBox } from './lib/geom.js';
import { RGBA } from './lib/color.js';
import Util from './lib/util.js';
import PortableFileSystem from './lib/filesystem.js';
import renderToString from './lib/preact-render-to-string.js';

let filesystem;

Util.colorCtor = ColoredText;

function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

const debug = (Util.getEnv('APP_ENV') + '').startsWith('devel');
/*||process.env.NODE_ENV.startsWith('devel')*/
async function testRenderSchematic(file) {
  let doc = new EagleDocument(filesystem.readFile(`${file}.sch`));
  //console.log('doc:', doc.get('eagle/drawing'));
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //console.log('renderer:', renderer);
  let output = renderer.render(doc, null, 0);
  console.log('console.log.filters:', console.log.filters);
  console.log('functionName:', Util.getStackFrame());

  //  throw new Error('test');
  //console.log('output:', output);
  //console.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  let outStr = renderToString(output, {}, { pretty: '  ' });
  dumpFile(`tmp/${outFile}.schematic.svg`, outStr);
  return outStr.length;
}

async function testRenderBoard(file) {
  let doc = new EagleDocument(filesystem.readFile(`${file}.brd`).toString(), null, `${file}.brd`);
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //console.log('renderer:', renderer);
  let output = renderer.render();

  console.log('output:', output);
  //console.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');
  let outStr = renderToString(output, {}, { pretty: '  ' });
  dumpFile(`tmp/${outFile}.board.svg`, outStr);
  return outStr.length;
}

async function main(...args) {
  await ConsoleSetup({ depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));
  //console.log.setFilters([/(test-rend|.*)/i]);
  //
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt');

  for(let filename of args) {
    //console.log('debug:', debug);

    let r = [await testRenderBoard(filename), await testRenderSchematic(filename)];
    console.log('r:', r);
  }

  let ct = new ColoredText();
  ct.write('this is a test ', new RGBA(255, 255, 0), new RGBA(0, 255, 255));
  ct.clearColors();
  ct.write('filename');
  ct.write(' = ', new RGBA(255, 255, 255), new RGBA(0, 0, 255));
  ct.write('blah.brd', new RGBA(0, 255, 255));

  ct.write(' arg', ' blah');

  //console.log( ct.toAnsi256());
  console.log(ct);
}

Util.callMain(main, true);

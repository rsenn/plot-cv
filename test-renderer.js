import { EagleDocument, Renderer } from './lib/eagle.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { render, Component } from './lib/preact.mjs';
import { ColoredText } from './lib/color/coloredText.js';
import { RGBA } from './lib/color.js';
import Util from './lib/util.js';
import PortableFileSystem from './lib/filesystem.js';

let filesystem;

Util.colorCtor = ColoredText;

const debug = Util.getEnv('APP_ENV').startsWith('devel');
/*||process.env.NODE_ENV.startsWith('devel')*/ async function testRenderSchematic(file) {
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

  let outStr = ReactComponent.toString(output);
  filesystem.writeFile(`tmp/${outFile}.schematic.svg`, outStr);
  return outStr.length;
}

async function testRenderBoard(file) {
  let doc = new EagleDocument((await fsPromises.readFile(`${file}.brd`)).toString());
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //console.log('renderer:', renderer);
  let output = renderer.render(doc, null, 0);

  //console.log('output:', output);
  //console.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');
  let outStr = ReactComponent.toString(output);
  filesystem.writeFile(`tmp/${outFile}.board.svg`, outStr);
  return outStr.length;
}

const filename = '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt';
async function main() {
  await ConsoleSetup();
  filesystem = await PortableFileSystem();
  console.log.setFilters([/(test-rend|.*)/i]);

  try {
    //console.log('debug:', debug);

    let r = [await testRenderSchematic(filename), await testRenderBoard(filename)];
    console.log('r:', r);

    let ct = new ColoredText();
    ct.write('this is a test ', new RGBA(255, 255, 0), new RGBA(0, 255, 255));
    ct.clearColors();
    ct.write('filename');
    ct.write(' = ', new RGBA(255, 255, 255), new RGBA(0, 0, 255));
    ct.write('blah.brd', new RGBA(0, 255, 255));

    ct.write(' arg', ' blah');

    //console.log( ct.toAnsi256());
    console.log(ct);
  } catch(error) {
    const stack = error.stack;

    /*console.log('argv[0]:', process.argv[0]);
      console.log('argv[1]:', Util.scriptDir());
      console.log('getURL():', Util.getURL());*/
    console.log(error.message, stack + '');
  }
}
main(Util.getArgs()).catch(error => {
  const stack = [...error.stack];
  console.log('ERROR:', error.message, stack);
});

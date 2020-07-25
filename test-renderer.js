import fs, { promises as fsPromises } from 'fs';
import { EagleDocument, Renderer } from './lib/eagle.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { render, Component } from './lib/preact.mjs';
import { ColoredText } from './lib/color/coloredText.js';
import { RGBA, HSLA } from './lib/color.js';

import Util from './lib/util.js';
import util from 'util';

Error.stackTraceLimit = 100;
Util.colorCtor = ColoredText;

const debug = (process.env.APP_ENV + '').startsWith('devel'); /*||process.env.NODE_ENV.startsWith('devel')*/

async function testRenderSchematic(file) {
  let doc = new EagleDocument((await fsPromises.readFile(`${file}.sch`)).toString());
  //Util.log('doc:', doc.get('eagle/drawing'));
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //Util.log('renderer:', renderer);
  let output = renderer.render(doc, null, 0);
  Util.log('Util.log.filters:', Util.log.filters);
  Util.log('functionName:', Util.getStackFrame());

  //  throw new Error('test');
  //Util.log('output:', output);
  //Util.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  let outStr = ReactComponent.toString(output);
  fs.writeFileSync(`tmp/${outFile}.schematic.svg`, outStr);
  return outStr.length;
}

async function testRenderBoard(file) {
  let doc = new EagleDocument((await fsPromises.readFile(`${file}.brd`)).toString());
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //Util.log('renderer:', renderer);
  let output = renderer.render(doc, null, 0);

  //Util.log('output:', output);
  //Util.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');
  let outStr = ReactComponent.toString(output);
  fs.writeFileSync(`tmp/${outFile}.board.svg`, outStr);
  return outStr.length;
}

const filename = '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt';
async function main() {
  Util.log.setFilters([/(test-rend|.*)/i]);

  try {
    //Util.log('debug:', debug);

    let r = [await testRenderSchematic(filename), await testRenderBoard(filename)];
    Util.log('r:', r);

    let ct = new ColoredText();
    ct.write('this is a test ', new RGBA(255, 255, 0), new RGBA(0, 255, 255));
    ct.clearColors();
    ct.write('filename');
    ct.write(' = ', new RGBA(255, 255, 255), new RGBA(0, 0, 255));
    ct.write('blah.brd', new RGBA(0, 255, 255));

    ct.write(' arg', ' blah');

    //Util.log( ct.toAnsi256());
    Util.log(ct);
  } catch(error) {
    const stack = error.stack;

    /*Util.log('argv[0]:', process.argv[0]);
      Util.log('argv[1]:', Util.scriptDir());
      Util.log('getURL():', Util.getURL());*/
    console.log(error.message, stack + '');
  }
}
main(process.argv.slice(2)).catch(error => {
  const stack = [...error.stack];
  console.log('ERROR:', error.message, stack);
});

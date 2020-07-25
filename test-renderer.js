import fs, { promises as fsPromises } from 'fs';
import { EagleDocument, Renderer } from './lib/eagle.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { render, Component } from './lib/preact.mjs';
import Util from './lib/util.js';
import util from 'util';

Error.stackTraceLimit = 100;

const debug = (process.env.APP_ENV + '').startsWith('devel'); /*||process.env.NODE_ENV.startsWith('devel')*/

async function testRenderSchematic(file) {
  let doc = new EagleDocument((await fsPromises.readFile(`${file}.sch`)).toString());
  //console.log('doc:', doc.get('eagle/drawing'));
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //console.log('renderer:', renderer);
  let output = renderer.render(doc, null, 0);
  console.log('functionName:', Util.getStackFrame());

  //  throw new Error('test');
  //console.log('output:', output);
  //console.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  let outStr = ReactComponent.toString(output);
  fs.writeFileSync(`tmp/${outFile}.schematic.svg`, outStr);
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
  fs.writeFileSync(`tmp/${outFile}.board.svg`, outStr);
  return outStr.length;
}

const filename = '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt';
async function main() {
  try {
    //console.log('debug:', debug);

    let r = [await testRenderSchematic(filename), await testRenderBoard(filename)];
    console.log('r:', r);
  } catch(error) {
    /*console.log('argv[0]:', process.argv[0]);
      console.log('argv[1]:', Util.scriptDir());
      console.log('getURL():', Util.getURL());*/
    console.log(Util.exception(error));

    console.log('stack:', Util.stack(error.stack));
  }
}
main(process.argv.slice(2)).catch(error => {
  const stack = [...error.stack];
  console.log('ERROR:', error.message, stack);
});

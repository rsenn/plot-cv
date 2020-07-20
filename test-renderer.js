import fs from 'fs';
import { EagleDocument, Renderer } from './lib/eagle.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { render, Component } from './lib/preact.mjs';

Error.stackTraceLimit = 100;

const debug = process.env.APP_ENV.startsWith('devel'); /*||process.env.NODE_ENV.startsWith('devel')*/

function testRenderSchematic(file) {
  let doc = new EagleDocument(fs.readFileSync(`${file}.sch`).toString());
  //console.log('doc:', doc.get('eagle/drawing'));
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //console.log('renderer:', renderer);
  let output = renderer.render(doc, null, 0);

  //console.log('output:', output);
  //console.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  fs.writeFileSync(`tmp/${outFile}.schematic.svg`, ReactComponent.toString(output));
}

function testRenderBoard(file) {
  let doc = new EagleDocument(fs.readFileSync(`${file}.brd`).toString());
  let renderer = new Renderer(doc, ReactComponent.append, debug);
  //console.log('renderer:', renderer);
  let output = renderer.render(doc, null, 0);

  //console.log('output:', output);
  //console.log('bounds:', doc.getBounds());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  fs.writeFileSync(`tmp/${outFile}.board.svg`, ReactComponent.toString(output));
}

const filename = '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt';

try {
  //console.log('debug:', debug);

  testRenderSchematic(filename);
  testRenderBoard(filename);
} catch(error) {
  //console.log('ERROR:', error.message);
  //console.log('stack:', error.stack);
}

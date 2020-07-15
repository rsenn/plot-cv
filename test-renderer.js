import fs from 'fs';
import { EagleDocument, SchematicRenderer, BoardRenderer, Renderer } from './lib/eagle.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { Circle } from './lib/geom/circle.js';
import { h, render, Component } from './node_modules/htm/preact/standalone.mjs';

  Error.stackTraceLimit = 100;

function testRenderSchematic(file) {
  let doc = (global.doc = new EagleDocument(fs.readFileSync(`${file}.sch`).toString()));
  //console.log('doc:', doc.get('eagle/drawing'));
  let renderer = new Renderer(doc, ReactComponent.append, true);
  let output = renderer.render(doc, null, 0);

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  fs.writeFileSync(`tmp/${outFile}.schematic.svg`, ReactComponent.toString(output));
}

function testRenderBoard(file) {
  let doc = (global.doc = new EagleDocument(fs.readFileSync(`${file}.brd`).toString()));
  let renderer = new Renderer(doc, ReactComponent.append, true);
  let output = renderer.render(doc, null, 0);

  //console.log('output:', output);

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  fs.writeFileSync(`tmp/${outFile}.board.svg`, ReactComponent.toString(output));
}

const filename = '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt';


try {
testRenderSchematic(filename);
testRenderBoard(filename);
}catch(error) {
  console.log("ERROR:", error.message);
  console.log("stack:", error.stack);
}

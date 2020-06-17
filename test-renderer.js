import fs from 'fs';
import { EagleDocument } from './lib/eagle/document.js';
import { SchematicRenderer, BoardRenderer, Renderer } from './lib/eagle/renderer.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { Circle } from './lib/geom/circle.js';
import { h, render, Component } from './node_modules/htm/preact/standalone.mjs';

function testRenderSchematic() {
  let doc = (global.doc = new EagleDocument(fs.readFileSync('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.sch').toString()));
  let renderer = new Renderer(doc, ReactComponent.append);
  let output = renderer.render(doc, null, 0);
  // console.log('output:', output);
  fs.writeFileSync('schematic.svg', ReactComponent.stringify(output));
}

function testRenderBoard() {
  let doc = (global.doc = new EagleDocument(fs.readFileSync('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString()));
  let renderer = new Renderer(doc, ReactComponent.append);
  let output = renderer.render(doc, null, 0);

  //console.log('output:', output);

  fs.writeFileSync('board.svg', ReactComponent.stringify(output));
}

testRenderSchematic();
testRenderBoard();

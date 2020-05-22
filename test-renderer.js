import fs from 'fs';
import { EagleDocument } from './lib/eagle/document.js';
import { SchematicRenderer } from './lib/eagle/renderer.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { h, render, Component } from './node_modules/htm/preact/standalone.mjs';

let doc = new EagleDocument(fs.readFileSync('Headphone-Amplifier-ClassAB-alt3.sch').toString());

console.log('bounds:', doc.getBounds());

let renderer = new SchematicRenderer(doc, (tag, attrs, parent) => {
  let elem = h(tag, attrs);

  if(parent) {
    if(parent.props.children instanceof Array) parent.props.children.push(elem);
    else if(parent.props.children) parent.props.children = [parent.props.children, elem];
    else parent.props.children = elem;
  }
  return elem;
});

//let svg = h('svg', {});
let output = renderer.render();

console.log('output:', output);

fs.writeFileSync('output.svg', ReactComponent.stringify(output));
//console.log("output:",ReactComponent.stringify(svg));

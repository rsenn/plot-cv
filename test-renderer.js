import fs from 'fs';
import { EagleDocument } from './lib/eagle/document.js';
import { SchematicRenderer } from './lib/eagle/renderer.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { Circle } from './lib/geom/circle.js';
import { h, render, Component } from './node_modules/htm/preact/standalone.mjs';

let doc = new EagleDocument(fs.readFileSync('Headphone-Amplifier-ClassAB-alt3.sch').toString());

//console.log('bounds:', doc.getBounds());

let renderer = new SchematicRenderer(doc, (tag, attrs, parent) => {
  let elem = h(tag, attrs);
  if(parent) {
    const { props } = parent;
    if(props.children instanceof Array) props.children.push(elem);
    else if(props.children) props.children = [props.children, elem];
    else props.children = elem;
  }
  return elem;
});

let output = renderer.render(null, null, 0);

//let instances = renderer.renderInstances();

console.log('output:', output);
/*
let circle = new Circle(0, 0, 10);

console.log('circle:', circle);
console.log('circle:', circle.bbox());*/

fs.writeFileSync('output.svg', ReactComponent.stringify(output));
//console.log("output:",ReactComponent.stringify(svg));

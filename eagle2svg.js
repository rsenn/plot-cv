#!/usr/bin/env qjsm
import * as path from './lib/path.js';
import { readFileSync } from 'fs';
import { log, EagleDocument, Renderer, EagleProject } from './lib/eagle.js';
import { Console } from 'console';
import { read as fromXML, write as toXML } from './lib/xml.js';
import { WriteFile } from './io-helpers.js';
import { getOpt } from 'util';
import renderToString from './lib/preact-render-to-string.js';
import { render } from './lib/preact.mjs';
import { append } from './lib/preact/append.js';

let debugFlag = false;

function render(doc, filename) {
  if(doc instanceof EagleProject) {
    render(doc.schematic);
    render(doc.board);
    return;
  }
  let renderer = new Renderer(doc, append, debugFlag);

  /* renderer.setPalette([
    [0xff, 0xff, 0xff],
    [0x4b, 0x4b, 0xa5],
    [0, 0, 0],
    [0x4b, 0xa5, 0xa5],
    [0, 0, 0],
    [0xa5, 0x4b, 0xa5],
    [0xa5, 0xa5, 0x4b],
    [0, 0, 0],
    [0x4b, 0x4b, 0xff],
    [0x4b, 0xff, 0x4b],
    [0x4b, 0xff, 0xff],
    [0xff, 0x4b, 0x4b],
    [0xff, 0x4b, 0xff],
    [0xff, 0xff, 0x4b],
    [0x4b, 0x4b, 0x4b],
    [0xa5, 0xa5, 0xa5],
    [0, 0, 0]
  ].map(([r,g,b]) => new RGBA(r,g,b)));*/

  let str;
  let svg = renderer.render(doc);
  try {
    str = renderToString(svg);
  } catch(e) {
    console.log('ERROR:', e);
    console.log('STACK:', e.stack);
  }
  let xml = fromXML(str);

  if(filename) {
    let ret;
    ret = WriteFile(filename, (str = toXML(xml)));
    console.log(`Saving to '${filename}'...`, ret);
  }
  return str;
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { maxArrayLength: 100, colors: true, depth: 2, compact: false, customInspect: true }
  });

  let params = getOpt(
    {
      debug: [false, value => (debugFlag = value), 'x'],
      'output-dir': [true, null, 'd'],
      '@': 'input'
    },
    args
  );

  for(let arg of args) {
    let doc = EagleDocument.open(arg, f => readFileSync(f, 'utf-8'));
    let file = path.basename(doc.filename, '.' + doc.type) + '-' + { sch: 'schematic', brd: 'board', lbr: 'library' }[doc.type] + '.svg';

    if(params['output-dir']) file = path.join(params['output-dir'], file);

    render(doc, file);
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}
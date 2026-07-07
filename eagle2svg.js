#!/usr/bin/env qjsm
import * as path from './lib/path.js';
import { readFileSync } from 'fs';
import { Console } from 'console';
import { getOpt } from 'util';
import { EagleDocument } from './eagle.js';
import { Renderer } from './eagle-renderer.js';
import renderToString from './lib/preact-render-to-string.js';
import { WriteFile } from './io-helpers.js';

const typeToExt = { schematic: 'sch', board: 'brd', library: 'lbr' };

function render(doc, filename) {
  const renderer = new Renderer(doc);
  const svg = renderer.render();
  let str;
  try {
    str = renderToString(svg);
  } catch(e) {
    console.log('ERROR:', e.message);
    console.log('STACK:', e.stack);
    return null;
  }
  if(filename) {
    const ret = WriteFile(filename, str);
    console.log(`Saving to '${filename}'...`, ret);
  }
  return str;
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { maxArrayLength: 100, colors: true, depth: 2, compact: false, customInspect: true },
  });

  const params = getOpt(
    {
      debug: [false, null, 'x'],
      'output-dir': [true, null, 'd'],
      '@': 'input',
    },
    args,
  );

  for(const arg of params['@'] || args) {
    const doc = EagleDocument.open(arg, f => readFileSync(f, 'utf-8'));
    const ext = typeToExt[doc.type] || doc.type;
    let file = path.basename(arg, '.' + ext) + '-' + doc.type + '.svg';
    if(params['output-dir']) file = path.join(params['output-dir'], file);
    render(doc, file);
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
}

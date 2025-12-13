import * as filesystem from 'fs';
import { ColoredText } from './lib/color/coloredText.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { EagleDocument, Renderer } from './lib/eagle.js';
import renderToString from './lib/preact-render-to-string.js';
import { render } from './lib/preact.js';
Util.colorCtor = ColoredText;

function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

const debug = (Util.getEnv('APP_ENV') + '').startsWith('devel');

async function testRenderSchematic(file) {
  let doc = new EagleDocument(filesystem.readFileSync(`${file}.sch`));

  let renderer = new Renderer(doc, ReactComponent.append, debug);

  let output = renderer.render(doc, null, 0);
  console.log('console.log.filters:', console.log.filters);
  console.log('functionName:', Util.getStackFrame());

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  let outStr = renderToString(output, {}, { pretty: '  ' });
  WriteFile(`tmp/${outFile}.schematic.svg`, outStr);
  return outStr.length;
}

async function testRenderBoard(file) {
  let doc = new EagleDocument(filesystem.readFileSync(`${file}.brd`).toString(), null, `${file}.brd`);
  let renderer = new Renderer(doc, ReactComponent.append, debug);

  let output = renderer.render();

  let outFile = file.replace(/.*\//g, '').replace(/\.[a-z]+$/, '');

  let outStr = renderToString(output, {}, { pretty: '  ' });
  WriteFile(`tmp/${outFile}.board.svg`, outStr);
  return outStr.length;
}

async function main(...args) {
  if(Util.platform == 'quickjs')
    await import('os').then(({ setTimeout, setInterval, clearInterval, clearTimeout }) => {
      Object.assign(globalThis, {
        setTimeout,
        setInterval,
        clearInterval,
        clearTimeout
      });
    });

  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt');

  for(let filename of args) {
    let r = [await testRenderBoard(filename), await testRenderSchematic(filename)];
    console.log('r:', r);
  }

  console.log('finished');
}

main().catch(err => console.log('error:', err));
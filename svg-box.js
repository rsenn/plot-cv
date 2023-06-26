import { read, write } from 'xml';
import { WriteFile, WriteFd, WriteClose, WriteAny, WriteJSON, WriteXML } from './io-helpers.js';
import { Rect } from './lib/geom/rect.js';

function main(...args) {
  let obj = {
    tagName: 'svg',
    attributes: {
      xmlns: 'http://www.w3.org/2000/svg',
      version: '1.1',
      viewBox: `0 0 210 297`,
      width: '210mm',
      height: '297mm'
    },
    children: []
  };

  let r = new Rect(0, 0, 100, 50);

  obj.children.push(r.toSVG(undefined, { stroke: 'black', 'stroke-width': 0.26458333, fill: 'none' }));

  WriteFile('output.svg', write(obj));
}

main(...scriptArgs.slice(1));

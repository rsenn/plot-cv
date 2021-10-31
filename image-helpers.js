import { WriteFile } from './io-helpers.js';
import * as xml from 'xml';


export function MakeSVG(children, size) {
  let viewBox = [0, 0, ...size].join(' ');
  return {
    tagName: 'svg',
    children: [{ tagName: 'g', attributes: { stroke: 'black' }, children }],
    attributes: {
      xmlns: 'http://www.w3.org/2000/svg',
      viewBox
    }
  };
}

export function SaveSVG(filename, doc) {
  const output = xml.write(doc);
  WriteFile(filename, output);
  console.log('Saved ' + filename + '.');
}

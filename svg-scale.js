import filesystem from 'fs';
import { Alea } from './lib/alea.js';
import deep from './lib/deep.js';
import { CSS } from './lib/dom.js';
import { Rect, Size } from './lib/geom.js';
import * as path from './lib/path.js';
import { parse as parsePath } from './lib/svg/path-parser.js';
import { toXML, tXml } from './lib/xml.js';

let prng = new Alea(Date.now());

function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFileSync(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}

function writeXML(filename, xml) {
  let str = toXML(xml) + '\n';
  let tempFileName = filename + '.' + prng.uint32();
  let ret = filesystem.writeFile(tempFileName, str) > 0 && filesystem.unlink(filename) == 0 && filesystem.rename(tempFileName, filename) == 0;
  if(ret) console.log(`Wrote '${filename}'.`);
  return ret;
}

function isPositive(n) {
  return typeof n == 'number' && Number.isFinite(n) && n > 0;
}

function parseStyle(styleStr) {
  let style = {};
  for(let entry of styleStr.split(/\s*;\s*/g)) {
    let index = entry.indexOf(':');
    let name = entry.substring(0, index).trim();
    let value = entry.substring(index + 1).trim();

    style[name] = value;
  }
  return style;
}

function* formatPath(path) {
  for(let part of path) {
    const { largeArc, relative, rx, ry, sweep, x, x1, x2, xAxisRotation, y, y1, y2 } = part;

    switch (part.code.toUpperCase()) {
      case 'M':
        yield `${part.code} ${x},${y}`;
        break;
      case 'L':
        yield `${part.code} ${x},${y}`;
        break;
      case 'H':
        yield `${part.code} ${x}`;
        break;
      case 'Z':
        yield part.code;
        break;
      case 'C':
        yield `${part.code} ${x1},${y1} ${x2},${y2} ${x},${y}`;
        break;
      case 'S':
        yield `${part.code} ${x2},${y2} ${x},${y}`;
        break;
      case 'Q':
        yield `${part.code} ${x1},${y1} ${x},${y}`;
        break;
      case 'T':
        yield `${part.code} ${x},${y}`;
        break;
      case 'A':
        yield `${part.code} ${rx} ${ry} ${xAxisRotation} ${largeArc ? 1 : 0} ${sweep ? 1 : 0} ${x},${y}`;
        break;
    }
  }
}

let props = [];

function flatSVG(svg) {
  return deep.flatten(svg, new Map(), (v, p) => typeof v == 'object' && 'tagName' in v);
}

function scaleSVG(file, size) {
  let svg = readXML(file);
  if(svg[0].tagName == '?xml') svg = svg[0].children;

  let defsIndex = svg[0].children.findIndex(e => e.tagName == 'defs');

  if(defsIndex > 0) svg[0].children.splice(0, defsIndex);

  let flat = flatSVG(svg);
  let css = new Map();
  let styleNodes = [];

  for(let [key, value] of flat) {
    let { tagName, attributes, children } = value;

    if(tagName.indexOf(':') != -1) {
      flat.delete(key);
      deep.unset(svg, key);
      continue;
    }

    if(tagName == 'style') {
      let styleSheet = CSS.parse(children.join('\n').trim());
      css = Util.merge(styleSheet, css);

      deep.set(svg, key + '.children', ['', ...CSS.format(styleSheet).trim().split(/\n/g), '']);

      styleNodes.push(key);
      continue;
      /* deep.unset(svg, key);
      continue;*/
    }

    if(attributes) {
      attributes = filter(attributes, (value, key) => key.indexOf(':') == -1);

      if('style' in attributes) {
        let style = parseStyle(attributes.style);
        Object.assign(attributes, style);
        delete attributes.style;
        console.log('style:', style);
      }

      if(tagName == 'path' && attributes.d) {
        let path = parsePath(attributes.d);
        attributes.d = [...formatPath(path)].join(' ');

        //console.log('path:', path.length);
      }

      if('id' in attributes) delete attributes.id;
    }

    flat.set(key, { tagName, attributes });
    deep.set(svg, key + '.tagName', tagName);
    deep.set(svg, key + '.attributes', attributes);
  }

  for(let [key, value] of flat) {
    let { tagName, attributes, children } = value;
    console.log('attributes', Object.keys(attributes));
    if(attributes && 'class' in attributes) {
      let className = attributes['class'];
      console.log('className', className);

      let styleDeclarations = CSS.match(css, `.${className}`);

      console.log('styleDeclarations', styleDeclarations);

      if(styleDeclarations.size) {
        let styles = Object.fromEntries([...styleDeclarations]);

        Object.assign(attributes, styles);

        delete attributes['class'];
      } else {
        Util.clear(styleNodes);
      }
      deep.set(svg, key + '.attributes', attributes);
    }
  }

  //  console.log('flat:', flatSVG(svg));
  console.log('css:', css);
  console.log('styleNodes:', styleNodes);
  styleNodes.forEach(path => deep.unset(svg, path));

  //console.log("svg:", svg);

  let attrs = svg[0].attributes;
  let { viewBox, width, height } = attrs;

  let viewRect = Rect.fromString(viewBox);
  let viewSize = width && height ? new Size(width, height) : viewRect.size;
  let aspect = viewSize.aspect();

  if(!isPositive(size.width)) size.width = size.height * aspect;

  if(!isPositive(size.height)) size.height = size.width / aspect;

  delete attrs.xmlns;
  delete attrs.version;
  Object.assign(attrs, size.toObject());

  //console.log('svg:', viewSize, aspect);
  console.log(size);
  //console.log('attributes:', svg[0].attributes);

  return writeXML(file, svg);
}

async function main(...args) {

  let size, arg;

  while((arg = args.shift())) {
    if(/^[-.\d]+[\*x][-.\d]+$/.test(arg)) {
      size = Size.fromString(arg);
      console.log(size);
      continue;
    }

    scaleSVG(arg, size);
  }
  props = Util.unique(props).sort();
  console.log('props:', props.join(', '));
}

main(...scriptArgs.slice(1));
import { EagleElement } from './lib/eagle/element.js';
import { EagleDocument } from './lib/eagle/document.js';
import { EagleProject } from './lib/eagle/project.js';
import { EagleLocator } from './lib/eagle/reference.js';
import Util from './lib/util.js';
import { inspect } from './lib/eagle/common.js';
function dump(o, depth = 2, breakLength = 400) {
  let s;
  if(o instanceof EagleElement) {
    s = inspect(o);
    depth * 4;
  } else s = util.inspect(o, { depth, colors: true, breakLength });
  return s;
}
function xmlize(obj, depth = 2) {
  return obj.toXML
    ? obj.toXML().replace(/>\s*</g, '>\n    <')
    : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}
function testLocator() {
  let testobj = [
    0,
    1,
    2,
    { name: 'roman', children: ['x', 'y', { id: 1, items: ['a', 'b', 'c'] }] }
  ];
  let l = new EagleLocator([3, 'children', 2, 'items', -2]);
  let a = [l.slice(), l.slice()];
  //Util.log('l:', dump(l));
  //Util.log('a[0] == a[1]:', a[0] === a[1]);
  a[1][0] = 'x';
  //Util.log('a:', dump(a));
  let b = [l.prevSibling, l.nextSibling, l.parent];
  //Util.log('b:', dump(b));
  //Util.log(b[2].parent.parent.up(3));
  //Util.log('apply:', l.apply(testobj));
}
async function testEagle(filename) {
  let proj = new EagleProject(filename);
  let { board, schematic, libraries } = proj;
  const getPackage = e => {
    const { document } = e;
    if(e.tagName == 'part') {
      const device = e.deviceset.find(v => v.tagName == 'device' && v.attributes.name == e.attributes.device,
        ([v]) => v
      );
      return device.package;
    }
    return e.package;
  };
  //Util.log(schematic.children);
  try {
    for(let e of proj.getAll(v => v.tagName == 'part' || v.tagName == 'element',
      ([v, l, h, d]) => new EagleElement(d, l, v)
    )) {
      //Util.log('proj:', dump(e, 1));
    }
  } catch(error) {
    const { stack } = error;
    //Util.log('error:', error.toString(), stack);
  }
  return;

  return proj.saveTo('.', true);
}
(async () => {
  try {
    await testLocator();
    await testEagle('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3').then(result =>
      Util.log(result)
    );
  } catch(err) {
    const stack = err.stack;

    //Util.log('err:', err.toString());
    //Util.log('stack:', stack);
    throw err;
  }
})();

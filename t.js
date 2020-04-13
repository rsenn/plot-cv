import fs from "fs";
import { EagleEntity } from "./lib/eagle/entity.js";
import { EagleDocument } from "./lib/eagle/document.js";
import { EagleProject } from "./lib/eagle/project.js";
import { EagleLocator } from "./lib/eagle/locator.js";
import Util from "./lib/util.js";
import util from "util";
import { Console } from "console";

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});

function dump(o, depth = 2, breakLength = 400) {
  if(o instanceof EagleEntity) {
    o = EagleEntity.dump(o);
    depth *= 4;
  }
  return util.inspect(o, { depth, colors: true, breakLength });
}

function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, ">\n    <") : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}

function testLocator() {
  let testobj = [0, 1, 2, { name: "roman", children: ["x", "y", { id: 1, items: ["a", "b", "c"] }] }];
  let l = new EagleLocator([3, "children", 2, "items", -2]);
  let a = [l.slice(), l.slice()];
  console.log("l:", dump(l));
  console.log("a[0] == a[1]:", a[0] === a[1]);
  a[1][0] = "x";
  console.log("a:", dump(a));
  let b = [l.prevSibling, l.nextSibling, l.parent];
  console.log("b:", dump(b));
  console.log(b[2].parent.parent.up(3));
  console.log("apply:", l.apply(testobj));
}

async function testEagle(filename) {
  let proj = new EagleProject(filename);
  let { board, schematic, libraries } = proj;

  const newEntity = ([v, l, h, d]) => {
    let e;

    let elem = d.index(l);
    console.log("", elem);
    e = new EagleEntity(d, l);
    return e;
  };

  const getText = ([v, l, h, d]) => {
    const { children, ...obj } = EagleEntity.toObject(v);
    return { ...obj, text: v.children.join(" ") };
  };

  let descriptions = [...schematic.getAll("description", getText)];
  let parts = [...schematic.getAll("part", ([v, l, h, d]) => new EagleEntity(d, l, v))];

  console.log(
    "parts:",
    parts.map(part => part.toXML())
  );

  let element0 = parts[0].children[0];

  return proj.saveTo(".", true);

  fs.writeSync(process.stdout.fd, "board:\n" + board.toString());

  elems.forEach(elem => {
    if(elem.deviceset) console.log(`${elem.tagName}.deviceset:`, elem.deviceset.toXML(1), elem.attributes.deviceset);
  });
  named.forEach(([e, p]) => {
    let elem = new EagleEntity(board, p);
    return elem;
  });
}

(async () => {
  try {
    await testLocator();
    await testEagle("../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3").then(result => console.log(result));
  } catch(err) {
    console.log("err:", err.toString());
  }
})();

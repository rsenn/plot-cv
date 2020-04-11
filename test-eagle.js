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

function dump(obj, depth = 2, breakLength = 400) {
  if(obj instanceof EagleEntity) {
    obj = EagleEntity.toObject(obj);
    depth *= 4;
  }
  return util.inspect(obj, { depth, colors: true, breakLength });
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

  /*console.log("libraries:", dump(proj.libraries));*/

  //let elements = [...board.getAll("element", (value, path, hier) => new EagleEntity(board, path, value))];
  let parts = [...schematic.getAll("part", (value, path, hier) => new EagleEntity(board, path, value))];
  let descriptions = [...schematic.getAll("description")];

  // console.log("descriptions:",descriptions.map(([value,path,hier]) => value));
  //  console.log("parts:\n  " + parts.map((part, i) => `part #${i}: ` + xmlize(part, 0)).join("\n  "));
  //  console.log("devices:\n  "+dump([...schematic.getAll("device",  (value,path,hier) => value /*new EagleEntity(board, path, value)*/)], 10, 200));
  //console.log("devicesets:\n" + [...schematic.getAll("deviceset", (value, path, hier, doc) => path)].join("\n"));

  let [elements] = board.find(p => p.tagName == "elements");
  console.log("elements:", dump((elements.children.map(EagleEntity.toObject)), 4));

  let element0 = elements.children[0];
  console.log("element0:", dump(EagleEntity.toObject(element0), 4));
  console.log("element0.keys:", EagleEntity.keys(element0), 4);
  console.log("element0.entries:", new Map(EagleEntity.entries(element0)), 4);
  let [value, path, hier] = board.find(el => el === element0);
  console.log("board.find:", { value, path, hier });

  return proj.saveTo(".", true);

  /*

  let named = [...board.findAll(e => e.attributes && "name" in e.attributes)];
  let withPackage = [...board.findAll(e => e.attributes && "device" in e.attributes)];
  let parents = [...board.findAll(e => e.children instanceof Array && e.children.length > 0)];
  let tags = named.map(e => E agleDocument.toXML(e));

  let nodes = EagleDocument.traverse(board.xml[0]);
*/
  //console.log("library", dump(libraries, 2));
  /*
  for(let [node, path, hier] of EagleDocument.traverse(board.xml[0])) {
    let pathstr = EagleDocument.nodeName(node, path, hier);
    yield [board.type + "/" + pathstr, EagleDocument.toXML(node, 0)];
  }
  return;*/

  fs.writeSync(process.stdout.fd, "board:\n" + board.toString());

  elems.forEach(elem => {
    //console.log(`${elem.tagName}:`, elem.toXML(0), elem.attributes.package);
    if(elem.deviceset) console.log(`${elem.tagName}.deviceset:`, elem.deviceset.toXML(1), elem.attributes.deviceset);
  });
  false &&
    named.forEach(([e, p]) => {
      let elem = new EagleEntity(board, p);
      //console.log("e:", elem.toXML(), elem.attributes.package);
    });
}

(async () => {
  try {
    await testLocator();
    await testEagle("../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3").then(result => console.log(result));
  } catch(err) {
    console.log("err:", err);
  }
})();

//for(let elem of result) console.log(elem.join("\t"));

import fs from "fs";
import { EagleEntity } from "./lib/eagle/entity.js";
import { EagleDocument } from "./lib/eagle/document.js";
import { EagleProject } from "./lib/eagle/project.js";
import Util from "./lib/util.js";
import util from "util";
import { Console } from "console";

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 20, colors: true }
});

function dump(obj, depth = 2) {
  return util.inspect(obj, { depth, colors: true });
}

function* testEagle(filename) {
  let children = fs.readFileSync(filename);
  //console.log("children:", children);

  let proj = new EagleProject(filename);
  let model = proj.board;
  let schematic = proj.schematic;

  console.log("schematic:", schematic);
  /*
    //console.log("tags:",[...model.tagNames()]);

  for(let [v, k, o, path] of Util.traverseWithPath(model.children)) {
    let obj = Util.indexByPath(model.children, path);

    if(!(v instanceof Array))
    //console.log("node:",);
  }
*/
  let elems = model.get("elements") || model.get("parts");

  for(let elem of elems) {
    //console.log("attributes:", elem.attributes);
    /*for(let prop in elem.attributes) {
    console.log("prop:", prop, dump(elem.attributes[prop]));
    }*/
    //console.log("children:", dump(elem.children));
  }
  let named = [...model.findAll(e => e.attributes && "name" in e.attributes)];
  let withPackage = [...model.findAll(e => e.attributes && "device" in e.attributes)];
  let parents = [...model.findAll(e => e.children instanceof Array && e.children.length > 0)];
  let tags = named.map(e => EagleDocument.toXML(e));
  let libraries = [
    ...model.findAll(e => e.tagName == "library" && e.attributes && "name" in e.attributes)
  ];

  let nodes = EagleDocument.traverse(model.xml[0]); /*.map((obj,path) => path)*/
  console.log("library", dump(libraries, 2));
  for(let [node, path, hier] of EagleDocument.traverse(model.xml[0])) {
    // if(hier.length <= 10)
    let pathstr = EagleDocument.nodeName(node, path, hier);

    yield model.type + "/" + pathstr + "\t" + EagleDocument.toXML(node, 0);
  }

  return;
  // console.log("named", dump(named, 3));
  //console.log("parents", dump(parents, 3));
  //console.log("tags", [...tags]);

  // console.log("elements", dump(elems,2));
  elems.forEach(elem => {
    console.log(`${elem.tagName}:`, elem.toXML(0), elem.attributes.package);

    if(elem.deviceset)
      console.log(`${elem.tagName}.deviceset:`, elem.deviceset.toXML(1), elem.attributes.deviceset);
  });

  false &&
    named.forEach(([e, p]) => {
      let elem = new EagleEntity(model, p);

      console.log("e:", elem.toXML(), elem.attributes.package);
      /*    console.log("obj.library:", dump(model.index(e.library.path), 1), "\ne.library:", dump(e.library, 1));
    if(e.attributes.deviceset) console.log("e.deviceset:", dump(model.index(e.deviceset.path), 1), "\ne.deviceset:", e.deviceset && dump(e.deviceset, 2));
*/ // if(e.children)
      //console.log("children:", dump(e.children, 0));
    });

  //console.log("element", dump(model.getByName("package", "CONN-3P")));
}

let result = [
  ...testEagle("/home/roman/Sources/an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.sch"),
  ...testEagle("/home/roman/Sources/an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd")
];

result = Util.unique(result.sort());
//console.log("result: ", result.join("\n"));

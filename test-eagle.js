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
  inspectOptions: { depth: 2, colors: true }
});

function dump(obj, depth = 2, breakLength = 400) {
  return util.inspect(obj, { depth, colors: true, breakLength });
}

function* testEagle(filename) {
  let proj = new EagleProject(filename);
  let { board, schematic, libraries } = proj;

  /*console.log("libraries:", dump(proj.libraries));*/


  let elems = board.get("elements") || board.get("parts");

  let named = [...board.findAll(e => e.attributes && "name" in e.attributes)];
  let withPackage = [...board.findAll(e => e.attributes && "device" in e.attributes)];
  let parents = [...board.findAll(e => e.children instanceof Array && e.children.length > 0)];
  let tags = named.map(e => EagleDocument.toXML(e));
/*
 libraries = [
    ...board.findAll(e => e.tagName == "library" && e.attributes && "name" in e.attributes)
  ];
*/
  let nodes = EagleDocument.traverse(board.xml[0]);

  //console.log("library", dump(libraries, 2));

  for(let [node, path, hier] of EagleDocument.traverse(board.xml[0])) {
    let pathstr = EagleDocument.nodeName(node, path, hier);
    yield [board.type + "/" + pathstr, EagleDocument.toXML(node, 0)];
  }
  return;

  elems.forEach(elem => {
    //console.log(`${elem.tagName}:`, elem.toXML(0), elem.attributes.package);
    if(elem.deviceset)
      console.log(`${elem.tagName}.deviceset:`, elem.deviceset.toXML(1), elem.attributes.deviceset);
  });
  false &&
    named.forEach(([e, p]) => {
      let elem = new EagleEntity(board, p);
      //console.log("e:", elem.toXML(), elem.attributes.package);
    });
}
let result = testEagle("../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3");

[...result];
//for(let elem of result) console.log(elem.join("\t"));

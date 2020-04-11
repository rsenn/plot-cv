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
  

  let proj = new EagleProject(filename);
  let model = proj.board;
  let schematic = proj.schematic;

  console.log("schematic:", schematic);
  
  let elems = model.get("elements") || model.get("parts");

  for(let elem of elems) {
    
    
    
  }
  let named = [...model.findAll(e => e.attributes && "name" in e.attributes)];
  let withPackage = [...model.findAll(e => e.attributes && "device" in e.attributes)];
  let parents = [...model.findAll(e => e.children instanceof Array && e.children.length > 0)];
  let tags = named.map(e => EagleDocument.toXML(e));
  let libraries = [
    ...model.findAll(e => e.tagName == "library" && e.attributes && "name" in e.attributes)
  ];

  let nodes = EagleDocument.traverse(model.xml[0]); 
  console.log("library", dump(libraries, 2));
  for(let [node, path, hier] of EagleDocument.traverse(model.xml[0])) {
    
    let pathstr = EagleDocument.nodeName(node, path, hier);

    yield model.type + "/" + pathstr + "\t" + EagleDocument.toXML(node, 0);
  }

  return;
  
  
  

  
  elems.forEach(elem => {
    console.log(`${elem.tagName}:`, elem.toXML(0), elem.attributes.package);

    if(elem.deviceset)
      console.log(`${elem.tagName}.deviceset:`, elem.deviceset.toXML(1), elem.attributes.deviceset);
  });

  false &&
    named.forEach(([e, p]) => {
      let elem = new EagleEntity(model, p);

      console.log("e:", elem.toXML(), elem.attributes.package);
       
      
    });

  
}

let result = [
  ...testEagle("/home/roman/Sources/an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.sch"),
  ...testEagle("/home/roman/Sources/an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd")
];

result = Util.unique(result.sort());



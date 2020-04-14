import fs from "fs";
import { EagleEntity } from "./lib/eagle/entity.js";
import { EagleDocument } from "./lib/eagle/document.js";
import { EagleProject } from "./lib/eagle/project.js";
import { EagleLocator } from "./lib/eagle/locator.js";
import Util from "./lib/util.js";
import util from "util";
import { Console } from "console";
import { ansi, text, dingbatCode, inspect, EagleNode } from "./lib/eagle/common.js";

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});
function dump(o, depth = 2, breakLength = 400) {
  let s;
  if(o instanceof EagleEntity) {
    s = inspect(o, undefined, { depth, location: false });
    depth * 4;
  } else s = util.inspect(o, { depth, colors: true, breakLength });
  return s;
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

  const getPackage = (d, e) => {
    let part;
    if(e.tagName == "instance") {
      part = e.part;
      let deviceset = part.deviceset;
      const device = deviceset.getByName("device", part.attributes.device);
      return device.package;
    }
    return e.package;
  };

  console.log("schematic:", schematic.getByName("instance", "T1", "part"));

  /*  console.log("board.location:", board.location);
  console.log("board.owner:", board.owner);*/
  /* const pred = v => (v.tagName == "instance" || v.tagName == "element") && "attributes" in v;
  const tran = ([v, l, d]) => new EagleEntity(d, l, v);*/
  const packages = {
    board: [...proj.board.getAll("package")],
    schematic: [...proj.schematic.getAll("package")]
  };

  for(let element of proj.board.getAll("element")) {
    let instance = proj.schematic.getByName("instance", element.attributes.name, "part");
    //console.log(dump(element,1));
    const pkgs = [element.package.name, instance.part.device.package.name];

    if(pkgs[0] != pkgs[1]) {
      console.log(dump(element, 1));
      console.log(dump(instance.part, 1));

      console.log(dump(pkgs, 1));

      const checkPkg = pkgName => {
        let bpkg = board.getByName("package", pkgName);
        let spkg = schematic.getByName("device", pkgName);

        if(!!(bpkg && spkg)) console.log(["board:     " + dump(bpkg, 1), "schematic: " + dump(spkg, 1)].join("\n"));
        return !!(bpkg && spkg);
      };
      let res = pkgs.map(pkg => checkPkg(pkg));

      let index = res.indexOf(true);

      if(index != -1) {
        instance.part.device = pkgs[index];
        console.log(instance.part.device, pkgs[index]);
        element.package = pkgs[index];
      }
      console.log(res, index);
    }
  }

  console.log("schematic:", schematic.changes);
  console.log("board:", board.changes);

  /*try {
    for(let e of Util.concat(proj.board.getAll(pred, tran), proj.schematic.getAll(pred, tran))) {
      const part = e.part;
      const deviceset = part && part.deviceset;
      const device = part && part.device;
      const pkg = e.package || device.package; // = getPackage(schematic, e);

      if(part) {
        console.log("device.attributes.package:", device.attributes.package);
        if(device.attributes.package && !pkg) console.log(`Package ${device.attributes.package} not found!`);
        console.log(dump(part, 1));
        console.log(dump(deviceset, 1));
        console.log(dump(device, 1));
      } else {
        console.log(dump(e, 1));
      }
      console.log(dump(pkg, 1));
    }
  } catch(error) {
    const { stack } = error;
    console.log(
      "error:",
      error
        .toString()
        .split(/\n/g)
        .slice(0, 10),
      stack
    );
    throw new Error("err");
  }*/
  return proj.saveTo(".", true);
}
(async () => {
  try {
    await testLocator();
    await testEagle("../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3").then(result => console.log(result));
  } catch(err) {
    const stack = err.stack;
    console.log("err:", err.toString());
    console.log("stack:", stack);
    throw err;
  }
})();
